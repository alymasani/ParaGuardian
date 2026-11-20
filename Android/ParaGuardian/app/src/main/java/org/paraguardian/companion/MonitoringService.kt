package org.paraguardian.companion

import android.Manifest
import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.pm.ServiceInfo
import android.os.Build
import android.os.IBinder
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.launch

class GuardianNotifications(private val context: Context) {
    private val manager = context.getSystemService(NotificationManager::class.java)
    private val content = PendingIntent.getActivity(context, 0, Intent(context, MainActivity::class.java),
        PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE)

    init {
        manager.createNotificationChannels(listOf(
            NotificationChannel("monitor", "Device monitoring", NotificationManager.IMPORTANCE_LOW),
            NotificationChannel("emergency", "Emergency and flight safety", NotificationManager.IMPORTANCE_HIGH)
        ))
    }

    fun ongoing(text: String = "Listening for the associated device"): Notification {
        val stop = PendingIntent.getService(context, 0, Intent(context, MonitoringService::class.java).setAction("stop"),
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE)
        return Notification.Builder(context, "monitor").setSmallIcon(android.R.drawable.ic_dialog_info)
            .setContentTitle("ParaGuardian monitoring").setContentText(text)
            .setContentIntent(content).setOngoing(true).setOnlyAlertOnce(true)
            .addAction(Notification.Action.Builder(null, "Stop", stop).build()).build()
    }

    fun updateMonitoring(text: String) {
        try { manager.notify(1, ongoing(text)) } catch (_: SecurityException) { }
    }

    fun alert(title: String, text: String, critical: Boolean): Boolean {
        if (!manager.areNotificationsEnabled() ||
            (Build.VERSION.SDK_INT >= 33 && !context.granted(Manifest.permission.POST_NOTIFICATIONS))) return false
        return try {
            manager.notify(if (critical) 3 else 2,
                Notification.Builder(context, if (critical) "emergency" else "monitor")
                    .setSmallIcon(android.R.drawable.ic_dialog_alert).setContentTitle(title).setContentText(text)
                    .setStyle(Notification.BigTextStyle().bigText(text)).setContentIntent(content)
                    .setAutoCancel(true).setOnlyAlertOnce(!critical).build())
            true
        } catch (_: SecurityException) { false }
    }
}

class MonitoringService : Service() {
    private var observation: Job? = null
    private val guardian get() = (application as GuardianApplication).guardian
    override fun onBind(intent: Intent?): IBinder? = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        val settings = Settings(this)
        if (intent?.action == "stop") {   // the user stopped monitoring from the notification
            guardian.setMonitoringRequested(false)
            stopSelf()
            return START_NOT_STICKY
        }
        // A null intent is Android re-creating this START_STICKY service after the process was killed:
        // resume from the persisted request instead of silently giving up.
        val address = intent?.getStringExtra("address") ?: settings.address?.takeIf { settings.monitoringRequested }
        if (address == null) {
            stopSelf()
            return START_NOT_STICKY
        }
        try {
            val notifications = GuardianNotifications(this)
            val notification = notifications.ongoing()
            if (Build.VERSION.SDK_INT >= 29)
                startForeground(1, notification, ServiceInfo.FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE)
            else startForeground(1, notification)
            if (!guardian.monitor(address)) {
                stopSelf()
                return START_NOT_STICKY
            }
            settings.recoveryPending = false
            observation?.cancel()
            observation = CoroutineScope(Dispatchers.Main.immediate).launch {
                guardian.state.map { it.link }.distinctUntilChanged().collect {
                    if (it == LinkState.PERMISSION) stopSelf()   // request kept: the UI shows "permission required"
                    else notifications.updateMonitoring(it.label)
                }
            }
        } catch (_: RuntimeException) {
            settings.recoveryPending = true
            guardian.banner("Android denied background monitoring. Open the app and tap Resume.")
            stopSelf()
            return START_NOT_STICKY
        }
        return START_STICKY
    }

    /** Swiping the app away from Recents does not end monitoring (it is a foreground service). */
    override fun onTaskRemoved(rootIntent: Intent?) = Unit

    override fun onDestroy() {
        observation?.cancel()
        guardian.stopMonitor()
        super.onDestroy()
    }
}
