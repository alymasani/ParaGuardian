package org.paraguardian.companion

import android.app.job.JobInfo
import android.app.job.JobParameters
import android.app.job.JobScheduler
import android.app.job.JobService
import android.companion.AssociationInfo
import android.companion.CompanionDeviceManager
import android.companion.CompanionDeviceService
import android.companion.DevicePresenceEvent
import android.companion.ObservingDevicePresenceRequest
import android.content.BroadcastReceiver
import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.pm.ApplicationInfo
import android.os.Build
import android.security.keystore.KeyGenParameterSpec
import android.security.keystore.KeyProperties
import android.util.Base64
import androidx.annotation.RequiresApi
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import java.security.KeyStore
import javax.crypto.Cipher
import javax.crypto.KeyGenerator
import javax.crypto.SecretKey
import javax.crypto.spec.GCMParameterSpec

/**
 * What the user sees about monitoring. REQUESTED (a persisted preference) and RUNNING (the foreground
 * service exists) are different facts; this derives one honest label from both, so a stopped service
 * is never shown as "monitoring active".
 */
enum class MonitorStatus(val label: String) {
    OFF("Monitoring off"),
    ACTIVE_CONNECTED("Monitoring · device connected"),
    ACTIVE_DISCONNECTED("Monitoring · device disconnected, reconnecting"),
    PERMISSION_REQUIRED("Monitoring stopped · Bluetooth permission required"),
    BLUETOOTH_OFF("Monitoring · Bluetooth is off"),
    RECOVERY_PENDING("Monitoring requested · waiting for the device to appear"),
    USER_ACTION_REQUIRED("Monitoring interrupted · open the app and tap Resume"),
}

fun monitorStatus(requested: Boolean, running: Boolean, link: LinkState, permissionOk: Boolean,
                  recoveryPending: Boolean, presenceObserved: Boolean): MonitorStatus = when {
    !requested -> MonitorStatus.OFF
    !permissionOk -> MonitorStatus.PERMISSION_REQUIRED
    running && link == LinkState.READY -> MonitorStatus.ACTIVE_CONNECTED
    running && link == LinkState.DISABLED -> MonitorStatus.BLUETOOTH_OFF
    running -> MonitorStatus.ACTIVE_DISCONNECTED
    recoveryPending -> MonitorStatus.USER_ACTION_REQUIRED
    presenceObserved -> MonitorStatus.RECOVERY_PENDING
    else -> MonitorStatus.USER_ACTION_REQUIRED
}

/** Starts the connected-device foreground service where Android allows it; reports when it does not. */
object MonitoringStarter {
    fun tryStart(context: Context, reason: String): Boolean {
        val settings = Settings(context)
        val address = settings.address
        if (!settings.monitoringRequested || address == null || !context.canConnect()) return false
        return try {
            context.startForegroundService(Intent(context, MonitoringService::class.java)
                .putExtra("address", address).putExtra("reason", reason))
            true
        } catch (_: RuntimeException) {
            // Android 12+: ForegroundServiceStartNotAllowedException (an IllegalStateException) outside the
            // allowed situations; SecurityException if a prerequisite permission was revoked.
            settings.recoveryPending = true
            GuardianNotifications(context).alert("ParaGuardian monitoring stopped",
                "Android did not allow monitoring to restart ($reason). Open ParaGuardian and tap Resume.", false)
            false
        }
    }
}

/**
 * BOOT_COMPLETED / MY_PACKAGE_REPLACED: restart monitoring if the user had requested it. A
 * connectedDevice foreground service may be started from BOOT_COMPLETED (the Android 15 boot
 * restriction covers dataSync, camera, mediaPlayback, phoneCall, mediaProjection and microphone).
 * Not received after a user "Force stop" until the user opens the app again (Android policy).
 */
class BootReceiver : BroadcastReceiver() {
    override fun onReceive(context: Context, intent: Intent) {
        if (intent.action != Intent.ACTION_BOOT_COMPLETED && intent.action != Intent.ACTION_MY_PACKAGE_REPLACED) return
        MonitoringStarter.tryStart(context, if (intent.action == Intent.ACTION_BOOT_COMPLETED) "phone restart" else "app update")
        DeliveryJobService.schedule(context, 0L)   // pending SOS submissions resume even without monitoring
    }
}

/**
 * Companion presence (Android 12+): the system binds this service when the associated ParaGuardian
 * device comes into Bluetooth range, which also exempts the app from the background foreground-service
 * start restriction for that moment. Used to resume monitoring after the device was out of range or
 * the process was killed.
 */
@RequiresApi(31)
class PresenceService : CompanionDeviceService() {
    @Deprecated("API 31-32 callback")
    override fun onDeviceAppeared(address: String) { MonitoringStarter.tryStart(this, "device appeared") }

    @Deprecated("API 33-35 callback")
    override fun onDeviceAppeared(associationInfo: AssociationInfo) {
        MonitoringStarter.tryStart(this, "device appeared")
    }

    @RequiresApi(36)
    override fun onDevicePresenceEvent(event: DevicePresenceEvent) {
        if (event.event == DevicePresenceEvent.EVENT_BLE_APPEARED) MonitoringStarter.tryStart(this, "device appeared")
    }
}

object PresenceObservation {
    /** Returns true if Android accepted the request to observe the associated device's presence. */
    fun start(context: Context, address: String): Boolean {
        if (Build.VERSION.SDK_INT < 31) return false
        val cdm = context.getSystemService(CompanionDeviceManager::class.java) ?: return false
        return try {
            if (Build.VERSION.SDK_INT >= 36) {
                val id = cdm.myAssociations.firstOrNull { it.deviceMacAddress?.toString().equals(address, true) }?.id
                    ?: return false
                cdm.startObservingDevicePresence(ObservingDevicePresenceRequest.Builder().setAssociationId(id).build())
            } else {
                @Suppress("DEPRECATION") cdm.startObservingDevicePresence(address)
            }
            true
        } catch (_: Exception) { false }   // not associated / not permitted: monitoring still works while open
    }

    fun stop(context: Context, address: String) {
        if (Build.VERSION.SDK_INT < 31) return
        val cdm = context.getSystemService(CompanionDeviceManager::class.java) ?: return
        try {
            if (Build.VERSION.SDK_INT >= 36) {
                val id = cdm.myAssociations.firstOrNull { it.deviceMacAddress?.toString().equals(address, true) }?.id
                    ?: return
                cdm.stopObservingDevicePresence(ObservingDevicePresenceRequest.Builder().setAssociationId(id).build())
            } else {
                @Suppress("DEPRECATION") cdm.stopObservingDevicePresence(address)
            }
        } catch (_: Exception) { }
    }
}

/**
 * Persisted JobScheduler job (survives reboot; RECEIVE_BOOT_COMPLETED) that drains the delivery outbox
 * whenever a network is available. Platform API: no extra dependency.
 */
class DeliveryJobService : JobService() {
    override fun onStartJob(params: JobParameters): Boolean {
        val guardian = (application as GuardianApplication).guardian
        CoroutineScope(Dispatchers.Main.immediate).launch {
            val next = try { guardian.runDeliveryNow() } catch (_: Exception) { 30_000L }
            jobFinished(params, false)
            if (next != null) schedule(this@DeliveryJobService, next)
        }
        return true
    }

    override fun onStopJob(params: JobParameters): Boolean = true   // reschedule with backoff

    companion object {
        private const val JOB_ID = 8001

        fun schedule(context: Context, delayMs: Long) {
            val scheduler = context.getSystemService(JobScheduler::class.java) ?: return
            val job = JobInfo.Builder(JOB_ID, ComponentName(context, DeliveryJobService::class.java))
                .setRequiredNetworkType(JobInfo.NETWORK_TYPE_ANY)
                .setMinimumLatency(delayMs.coerceIn(0L, RemoteDelivery.MAX_BACKOFF_MS))
                .setPersisted(true)
                .build()
            try { scheduler.schedule(job) } catch (_: RuntimeException) { }
        }
    }
}

/**
 * Backend credentials. The per-device secret is encrypted with an AES-256-GCM key held in the Android
 * Keystore (non-exportable); only ciphertext is stored in app-private preferences (backup disabled).
 */
class BackendCredentials(private val context: Context) {
    private val prefs = context.getSharedPreferences("backend_v1", Context.MODE_PRIVATE)

    val debugLoopbackAllowed get() = context.applicationInfo.flags and ApplicationInfo.FLAG_DEBUGGABLE != 0

    fun load(): BackendConfig? {
        val url = prefs.getString("url", null) ?: return null
        val device = prefs.getString("device", null) ?: return null
        val blob = prefs.getString("secret", null) ?: return null
        val secret = try {
            val raw = Base64.decode(blob, Base64.NO_WRAP)
            val cipher = Cipher.getInstance("AES/GCM/NoPadding")
            cipher.init(Cipher.DECRYPT_MODE, key(), GCMParameterSpec(128, raw.copyOfRange(0, 12)))
            cipher.doFinal(raw, 12, raw.size - 12)
        } catch (_: Exception) { return null }
        return BackendConfig(url, device, secret).takeIf { it.valid(debugLoopbackAllowed) }
    }

    fun save(url: String, device: String, secretHex: String): Boolean {
        val secret = secretHex.trim().takeIf { it.matches(Regex("([0-9a-fA-F]{2}){16,64}")) }
            ?.chunked(2)?.map { it.toInt(16).toByte() }?.toByteArray() ?: return false
        if (!BackendConfig(url.trim(), device.trim(), secret).valid(debugLoopbackAllowed)) return false
        val cipher = Cipher.getInstance("AES/GCM/NoPadding")
        cipher.init(Cipher.ENCRYPT_MODE, key())
        val blob = cipher.iv + cipher.doFinal(secret)
        return commit { putString("url", url.trim()); putString("device", device.trim())
            putString("secret", Base64.encodeToString(blob, Base64.NO_WRAP)) }
    }

    fun clear(): Boolean = commit { clear() }

    /** KTX edit {} discards commit()'s result; the caller must know whether the credential was stored. */
    @android.annotation.SuppressLint("UseKtx", "ApplySharedPref")
    private fun commit(block: android.content.SharedPreferences.Editor.() -> Unit): Boolean =
        prefs.edit().apply(block).commit()

    private fun key(): SecretKey {
        val store = KeyStore.getInstance("AndroidKeyStore").apply { load(null) }
        (store.getKey(ALIAS, null) as? SecretKey)?.let { return it }
        val generator = KeyGenerator.getInstance(KeyProperties.KEY_ALGORITHM_AES, "AndroidKeyStore")
        generator.init(KeyGenParameterSpec.Builder(ALIAS, KeyProperties.PURPOSE_ENCRYPT or KeyProperties.PURPOSE_DECRYPT)
            .setBlockModes(KeyProperties.BLOCK_MODE_GCM).setEncryptionPaddings(KeyProperties.ENCRYPTION_PADDING_NONE)
            .setKeySize(256).build())
        return generator.generateKey()
    }

    private companion object { const val ALIAS = "pg_backend_secret_v1" }
}
