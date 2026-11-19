package org.paraguardian.companion

import android.Manifest
import android.app.Activity
import android.bluetooth.BluetoothDevice
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.companion.AssociationInfo
import android.companion.AssociationRequest
import android.companion.BluetoothLeDeviceFilter
import android.companion.CompanionDeviceManager
import android.content.Intent
import android.content.IntentSender
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.os.ParcelUuid
import android.os.SystemClock
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.IntentSenderRequest
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.Checkbox
import androidx.compose.material3.FilterChip
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableLongStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.lifecycleScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import java.time.Instant
import java.util.Locale

class MainActivity : ComponentActivity() {
    private val guardian get() = (application as GuardianApplication).guardian
    private val permissions = registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { grants ->
        if (grants.values.any { !it }) guardian.banner("Permission declined. Related features may be unavailable.")
        else guardian.banner("Permissions updated.")
    }
    private val association = registerForActivityResult(ActivityResultContracts.StartIntentSenderForResult()) { result ->
        if (result.resultCode != Activity.RESULT_OK) {
            guardian.banner("Association did not complete.")
            return@registerForActivityResult
        }
        if (Build.VERSION.SDK_INT < 33) {
            @Suppress("DEPRECATION")
            val item = result.data?.getParcelableExtra<android.os.Parcelable>(CompanionDeviceManager.EXTRA_DEVICE)
            val device = when (item) {
                is ScanResult -> item.device
                is BluetoothDevice -> item
                else -> null
            }
            if (device != null) selected(device.address)
        }
    }
    private var draftPending = false

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            MaterialTheme(colorScheme = lightColorScheme(
                primary = Color(0xFF006A5B), secondary = Color(0xFF42635B),
                tertiary = Color(0xFF335F83), background = Color(0xFFF5F9F7)
            )) {
                val state by guardian.state.collectAsStateWithLifecycle()
                GuardianScreen(state)
            }
        }
    }

    override fun onStart() {
        super.onStart()
        guardian.activityVisible(true)
    }

    override fun onStop() {
        guardian.activityVisible(false)
        super.onStop()
    }

    private fun stopMonitoring() {
        val service = Intent(this, MonitoringService::class.java)
        stopService(service)
    }

    private fun requestBluetooth() {
        if (Build.VERSION.SDK_INT >= 31)
            permissions.launch(arrayOf(Manifest.permission.BLUETOOTH_CONNECT, Manifest.permission.BLUETOOTH_SCAN))
        else guardian.banner("Use Android’s companion device picker to scan and associate.")
    }

    private fun requestNotifications() {
        if (Build.VERSION.SDK_INT >= 33) permissions.launch(arrayOf(Manifest.permission.POST_NOTIFICATIONS))
        else startActivity(Intent(android.provider.Settings.ACTION_APP_NOTIFICATION_SETTINGS)
            .putExtra(android.provider.Settings.EXTRA_APP_PACKAGE, packageName))
    }

    private fun pickDevice() {
        val manager = getSystemService(CompanionDeviceManager::class.java)
        val scan = ScanFilter.Builder().setServiceUuid(ParcelUuid(GattProfile.SERVICE)).build()
        val filter = BluetoothLeDeviceFilter.Builder().setScanFilter(scan).build()
        val request = AssociationRequest.Builder().addDeviceFilter(filter).build()
        val callback = object : CompanionDeviceManager.Callback() {
            @Deprecated("Used by Android 12 and earlier")
            override fun onDeviceFound(chooserLauncher: IntentSender) {
                association.launch(IntentSenderRequest.Builder(chooserLauncher).build())
            }

            override fun onAssociationPending(intentSender: IntentSender) {
                association.launch(IntentSenderRequest.Builder(intentSender).build())
            }

            override fun onAssociationCreated(info: AssociationInfo) {
                if (Build.VERSION.SDK_INT >= 33) info.deviceMacAddress?.toString()?.let(::selected)
            }

            override fun onFailure(error: CharSequence?) {
                guardian.banner("Android could not find or associate a ParaGuardian peripheral.")
            }
        }
        try {
            if (Build.VERSION.SDK_INT >= 33) manager.associate(request, mainExecutor, callback)
            else {
                @Suppress("DEPRECATION")
                manager.associate(request, callback, null)
            }
        } catch (_: RuntimeException) {
            guardian.banner("Companion pairing unavailable. Check Bluetooth and Android permissions.")
        }
    }

    private fun selected(address: String) {
        guardian.associate(address)
        guardian.banner("Associated in Android. Device authorization is pending firmware provisioning.")
    }

    private fun forgetDevice(address: String?) {
        if (address == null) return
        try {
            @Suppress("DEPRECATION")
            getSystemService(CompanionDeviceManager::class.java).disassociate(address)
        } catch (_: RuntimeException) {
            guardian.banner("Android association could not be removed. Check system device settings.")
        }
        guardian.associate(null)
    }

    private fun startMonitoring(address: String?) {
        if (address == null) return guardian.banner("Associate a device first.")
        if (!canConnect()) return guardian.banner("Grant Bluetooth permission first.")
        try {
            startForegroundService(Intent(this, MonitoringService::class.java).putExtra("address", address))
        } catch (_: RuntimeException) {
            guardian.banner("Monitoring was denied. Restart it while the app is visible.")
        }
    }

    private fun preparePhoneSos() {
        if (draftPending) return
        draftPending = true
        lifecycleScope.launch {
            try { guardian.phoneSos(visiblePhoneLocation(this@MainActivity, true)) }
            finally { draftPending = false }
        }
    }

    private fun composeSms(number: String, body: String): Boolean {
        return try {
            startActivity(Intent(Intent.ACTION_SENDTO, Uri.fromParts("smsto", number, null)).putExtra("sms_body", body))
            true
        } catch (_: android.content.ActivityNotFoundException) { false }
    }

    @OptIn(ExperimentalLayoutApi::class)
    @Composable
    private fun GuardianScreen(s: ViewState) {
        val tabs = listOf("Setup", "Device", "Contacts", "Emergency", "History", "Settings")
        var tab by rememberSaveable { mutableStateOf("Setup") }
        var elapsed by remember { mutableLongStateOf(SystemClock.elapsedRealtime()) }
        LaunchedEffect(Unit) {
            while (true) { delay(300); elapsed = SystemClock.elapsedRealtime() }
        }
        val status = s.status?.takeIf { elapsed - s.statusAt <= 2_000 }
        Scaffold { insets ->
            Column(Modifier.padding(insets).padding(horizontal = 20.dp)) {
                Spacer(Modifier.height(12.dp))
                Text("ParaGuardian", style = MaterialTheme.typography.headlineMedium)
                Text(if (s.simulation) "Simulation · No external messages"
                    else if (s.monitoring) s.link.label else "Physical mode · Monitoring inactive",
                    color = MaterialTheme.colorScheme.primary)
                FlowRow(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                    tabs.forEach { name -> FilterChip(selected = tab == name, onClick = { tab = name },
                        label = { Text(name) }) }
                }
                if (s.message.isNotBlank()) Text(s.message, style = MaterialTheme.typography.bodySmall)
                HorizontalDivider(Modifier.padding(vertical = 8.dp))
                Column(Modifier.weight(1f).verticalScroll(rememberScrollState()).padding(vertical = 8.dp),
                    verticalArrangement = Arrangement.spacedBy(10.dp)) {
                    if (!s.storageReady) Text("Loading local data. Check status if storage remains unavailable.")
                    when (tab) {
                        "Setup" -> Setup(s)
                        "Device" -> Device(s, status)
                        "Contacts" -> Contacts(s)
                        "Emergency" -> Emergency(s, status, elapsed)
                        "History" -> History(s)
                        "Settings" -> SettingsPanel(s)
                    }
                    Spacer(Modifier.height(16.dp))
                }
            }
        }
    }

    @Composable
    private fun Setup(s: ViewState) {
        Title("Set up your companion")
        Text("ParaGuardian can receive suspected-crash and manual SOS alerts. Simulation exercises the complete workflow with a mock provider.")
        Text("Physical Bluetooth requires a provisioned peripheral. SMS requires you to confirm send in your messaging app.")
        Toggle("Simulation mode", s.simulation, s.storageReady && !s.monitoring, guardian::mode)
        if (s.simulation) {
            Button(onClick = { guardian.simulationConnect(!s.simulatedConnected) }) {
                Text(if (s.simulatedConnected) "Disconnect simulator" else "Connect simulator")
            }
        } else {
            Button(onClick = ::requestBluetooth) { Text("Bluetooth permissions") }
            Button(onClick = ::pickDevice, enabled = s.storageReady && !s.monitoring) { Text("Find and associate device") }
            Text(if (s.address == null) "No device selected" else "Android association saved")
            Button(onClick = {
                if (s.monitoring) stopMonitoring() else startMonitoring(s.address)
            }, enabled = s.monitoring || (s.storageReady && s.address != null)) {
                Text(if (s.monitoring) "Stop monitoring" else "Start monitoring")
            }
            Text("SMS contacts ready · ${s.contacts.count { it.eligible }}")
        }
        OutlinedButton(onClick = ::requestNotifications) { Text("Notification permission") }
    }

    @Composable
    private fun Device(s: ViewState, status: Protocol.DeviceStatus?) {
        Title("Device dashboard")
        Text(if (s.simulation) if (s.simulatedConnected) "Simulator connected" else "Simulator disconnected" else s.link.label)
        Text("Last authenticated or simulated connection · ${s.lastConnected?.let(Instant::ofEpochMilli) ?: "—"}")
        if (status == null) Text(if (s.status != null) "Telemetry is stale" else "Waiting for device telemetry")
        else {
            Text("Flight · ${flightPhase(status.phase)}")
            Text("Navigation · ${navigationState(status.nav)}")
            Text("Relative altitude · ${status.altitude?.let { format(it, "m") } ?: "—"}")
            Text("Vertical speed · ${status.speed?.let { format(it, "m/s") } ?: "—"}")
            Text("Battery · ${status.batteryMv?.let { "$it mV" } ?: "Unknown"}")
            Text("Sensor quality flags · 0x${status.quality.toString(16)}")
            Text("Safety alerts · ${warnings(status.warnings)}")
            Text("Emergency · ${deviceState(status)}")
        }
        OutlinedButton(onClick = { guardian.command(4) },
            enabled = s.simulatedConnected || s.link == LinkState.READY) { Text("Request device status") }
    }

    @Composable
    private fun Contacts(s: ViewState) {
        Title("Emergency contacts")
        Text("Route · User-confirmed SMS. Enter an international number and include only people who have agreed to receive alerts.")
        var editing by rememberSaveable { mutableStateOf<Int?>(null) }
        s.contacts.forEachIndexed { index, contact ->
            Card(Modifier.fillMaxWidth()) {
                Column(Modifier.padding(12.dp)) {
                    Text(contact.name, style = MaterialTheme.typography.titleMedium)
                    Text("${contact.number} · ${if (contact.eligible) "Ready" else "Excluded"}")
                    Row {
                        TextButton(onClick = { editing = index }) { Text("Edit") }
                        TextButton(onClick = { guardian.saveContacts(s.contacts.filterIndexed { i, _ -> i != index }) }) {
                            Text("Remove")
                        }
                    }
                }
            }
        }
        Button(onClick = { editing = s.contacts.size }, enabled = s.storageReady && s.contacts.size < 5) {
            Text("Add contact")
        }
        editing?.let { index ->
            val existing = s.contacts.getOrNull(index)
            var name by rememberSaveable(index) { mutableStateOf(existing?.name.orEmpty()) }
            var number by rememberSaveable(index) { mutableStateOf(existing?.number.orEmpty()) }
            var enabled by rememberSaveable(index) { mutableStateOf(existing?.enabled ?: true) }
            var consent by rememberSaveable(index) { mutableStateOf(existing?.consent ?: false) }
            val contact = Contact(name.trim(), number.trim(), enabled, consent)
            AlertDialog(onDismissRequest = { editing = null }, title = { Text("SMS contact") },
                text = {
                    Column(Modifier.verticalScroll(rememberScrollState())) {
                        OutlinedTextField(name, { name = it.take(80) }, label = { Text("Display name") }, singleLine = true)
                        OutlinedTextField(number, { number = it.take(16) }, label = { Text("Number with country code") }, singleLine = true)
                        Toggle("Enabled", enabled, true) { enabled = it }
                        Toggle("Approved to receive SOS", consent, true) { consent = it }
                    }
                },
                confirmButton = {
                    TextButton(enabled = contact.valid && s.contacts.filterIndexed { i, _ -> i != index }
                        .none { it.number == contact.number }, onClick = {
                        guardian.saveContacts(s.contacts.toMutableList().apply {
                            if (index == size) add(contact) else set(index, contact)
                        })
                        editing = null
                    }) { Text("Save") }
                }, dismissButton = { TextButton(onClick = { editing = null }) { Text("Cancel") } })
        }
        val preview = s.records.firstOrNull { it.route == if (s.simulation) Route.SIMULATION else Route.SMS }
            ?: Record(Key("preview", 0), 1, 2, System.currentTimeMillis(), System.currentTimeMillis(),
                if (s.simulation) Route.SIMULATION else Route.SMS, null, null, Stage.READY)
        Text("Message preview", style = MaterialTheme.typography.titleMedium)
        Text(emergencyMessage(preview, s.template, System.currentTimeMillis()))
    }

    @OptIn(ExperimentalLayoutApi::class)
    @Composable
    private fun Emergency(s: ViewState, status: Protocol.DeviceStatus?, elapsed: Long) {
        Title("Emergency status")
        var confirm by rememberSaveable { mutableStateOf<String?>(null) }
        val connected = s.simulatedConnected || s.link == LinkState.READY
        if (status != null && status.state != 0) {
            Text("Device incident #${status.incident} · ${deviceState(status)}",
                style = MaterialTheme.typography.titleLarge, color = MaterialTheme.colorScheme.error)
            Text("Trigger · ${trigger(status.trigger)}")
            if (status.state == 1) {
                val remaining = (status.countdownMs - (elapsed - s.statusAt).coerceAtLeast(0)).coerceAtLeast(0)
                Text("Countdown · ${(remaining + 999) / 1000} s")
                OutlinedButton(onClick = { guardian.command(2, status.incident) }, enabled = connected && remaining > 0) {
                    Text("Cancel pending SOS")
                }
            }
            Text("Attempts · ${status.attempts}  •  Cycle · ${status.cycles}")
            if (status.retryMs > 0) Text("Retry scheduled in ${(status.retryMs + 999) / 1000} s")
            if (status.state == 8 && status.cancelNotice != 0) Text("A prior send may have occurred. Cancellation cannot recall it.")
        } else Text(if (connected) "No active device incident reported" else "No live device status")
        Button(onClick = { confirm = "device" }, enabled = connected) { Text("Manual device SOS") }
        if (connected) OutlinedButton(onClick = { guardian.command(3, status?.incident ?: 0) }) { Text("Acknowledge alert sound") }
        if (!s.simulation) {
            Button(onClick = { confirm = "phone" }, enabled = s.storageReady && s.contacts.any { it.eligible }) {
                Text("Prepare phone SOS")
            }
            OutlinedButton(onClick = {
                permissions.launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION, Manifest.permission.ACCESS_FINE_LOCATION))
            }) { Text("Phone location permission") }
            OutlinedButton(onClick = {
                lifecycleScope.launch {
                    val fix = visiblePhoneLocation(this@MainActivity, false)
                    guardian.cachePosition(fix)
                    guardian.banner(if (fix == null) "Phone location unavailable." else "Phone fix cached with its timestamp.")
                }
            }) { Text("Refresh phone location") }
        }
        val recent = s.records.firstOrNull { it.route == if (s.simulation) Route.SIMULATION else Route.SMS }
        if (recent != null) {
            HorizontalDivider()
            Text("Phone record #${recent.key.incident} · ${recent.stage.label}",
                style = MaterialTheme.typography.titleMedium)
            if (recent.cancelledNotice) Text("Device cancellation recorded. Review any prior dispatch.")
            Text(emergencyMessage(recent, s.template, System.currentTimeMillis()))
            if (!s.simulation && !recent.cancelledNotice && (recent.stage == Stage.READY ||
                    (recent.stage == Stage.UNKNOWN && recent.composerTargets.isNotEmpty()))
            ) s.contacts.filter { it.eligible }.forEach { contact ->
                val opened = contactTarget(contact.number) in recent.composerTargets
                OutlinedButton(onClick = { guardian.openComposer(recent.key, contact, ::composeSms) }, enabled = !opened) {
                    Text(if (opened) "Composer opened for ${contact.name}" else "Compose SMS to ${contact.name}")
                }
            }
        }
        confirm?.let { mode ->
            AlertDialog(onDismissRequest = { confirm = null }, title = { Text("Confirm manual SOS") },
                text = { Text(if (mode == "device") "Queue a manual SOS on the connected device."
                    else "Prepare a draft for a configured contact. Confirm send separately in your SMS app.") },
                confirmButton = {
                    TextButton(onClick = {
                        confirm = null
                        if (mode == "device") guardian.command(1, status?.incident?.takeIf { status.state != 0 } ?: 0)
                        else preparePhoneSos()
                    }) { Text("Continue") }
                }, dismissButton = { TextButton(onClick = { confirm = null }) { Text("Cancel") } })
        }
        if (s.simulation) {
            HorizontalDivider()
            Text("Simulation controls", style = MaterialTheme.typography.titleMedium)
            FlowRow(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                listOf("Flight", "Suspect crash", "Advance 31 seconds", "Repeat SOS", "Corrupt next SOS",
                    "Toggle GNSS", "Toggle report loss", "Toggle report delay", "Toggle network",
                    "Simulate provider accepted", "Simulate delivered",
                    "Simulate failed", "Simulate unknown", "Restart incident engine", "New simulated device").forEach { action ->
                    OutlinedButton(onClick = { guardian.simulationAction(action) }) { Text(action) }
                }
            }
        }
    }

    @Composable
    private fun History(s: ViewState) {
        Title("History and diagnostics")
        s.records.forEach { record ->
            Text("${record.route.name} #${record.key.incident} · ${record.stage.label} · ${Instant.ofEpochMilli(record.received)}")
            Text("Revision ${record.revision} · ${if (record.attempted) "Attempt recorded" else "No automatic attempt"}",
                style = MaterialTheme.typography.bodySmall)
            s.timeline[record.key].orEmpty().take(8).forEach { event ->
                Text("${Instant.ofEpochMilli(event.at)} · r${event.revision} · " +
                    if (event.cancelled) "Cancellation notice" else event.stage.label,
                    style = MaterialTheme.typography.bodySmall)
            }
        }
        if (s.records.isEmpty()) Text("No stored incidents")
        HorizontalDivider()
        s.events.forEach { Text(it, style = MaterialTheme.typography.bodySmall) }
        OutlinedButton(onClick = {
            val diagnostics = buildString {
                appendLine("ParaGuardian technical diagnostics")
                appendLine("Mode ${if (s.simulation) "simulation" else "physical"} • ${s.link.label}")
                appendLine("Bluetooth permission ${canConnect()} · monitoring ${s.monitoring}")
                appendLine("Incident counts by stage ${s.records.groupingBy { it.stage }.eachCount()}")
                s.status?.let {
                    appendLine("Device health flags ${it.quality} · navigation ${it.nav ?: "unknown"}")
                }
                s.events.forEach { appendLine(it) }
            }
            startActivity(Intent.createChooser(Intent(Intent.ACTION_SEND).setType("text/plain")
                .putExtra(Intent.EXTRA_TEXT, diagnostics), "Export redacted diagnostics"))
        }) { Text("Export redacted diagnostics") }
    }

    @Composable
    private fun SettingsPanel(s: ViewState) {
        Title("Settings")
        Toggle("Flight safety notifications", s.safetyNotifications, s.storageReady, guardian::safetyNotifications)
        Toggle("Test mode", s.simulation, s.storageReady && !s.monitoring, guardian::mode)
        var template by rememberSaveable(s.template) { mutableStateOf(s.template) }
        OutlinedTextField(template, { template = it.take(160) }, label = { Text("Optional message footer") })
        Button(onClick = { guardian.template(template) }, enabled = s.storageReady && template != s.template) {
            Text("Save message footer")
        }
        Text(if (s.simulation) "Delivery route · In-app mock provider" else "Delivery route · User-confirmed SMS")
        OutlinedButton(onClick = { forgetDevice(s.address) }, enabled = s.address != null && !s.monitoring) {
            Text("Remove Android association")
        }
        Text("Incident content is retained for up to 30 days. Clearing history removes visible details and retains delivery deduplication markers.")
        var clear by remember { mutableStateOf(false) }
        OutlinedButton(onClick = { clear = true }, enabled = s.storageReady) { Text("Clear local history") }
        if (clear) AlertDialog(onDismissRequest = { clear = false }, title = { Text("Clear local incident history?") },
            confirmButton = { TextButton(onClick = { clear = false; guardian.clearHistory() }) { Text("Clear") } },
            dismissButton = { TextButton(onClick = { clear = false }) { Text("Keep") } })
        Text("Troubleshooting · Check Bluetooth and notification permissions. Reopen the app to restart monitoring after reboot or process termination.")
    }
}

@Composable private fun Title(text: String) = Text(text, style = MaterialTheme.typography.titleLarge)

@Composable
private fun Toggle(label: String, checked: Boolean, enabled: Boolean, toggle: (Boolean) -> Unit) {
    Row(verticalAlignment = androidx.compose.ui.Alignment.CenterVertically) {
        Checkbox(checked, toggle, enabled = enabled)
        Text(label)
    }
}

private fun format(value: Double, unit: String) = String.format(Locale.US, "%.1f %s", value, unit)

private fun deviceState(status: Protocol.DeviceStatus): String {
    if (status.state == 7 && status.failure == 1) return "Automatic retry budget exhausted · unresolved"
    if (status.state == 2 && status.flags and 8L != 0L) return "Waiting for connection"
    return listOf("Armed", "SOS countdown", "SOS pending", "Awaiting phone acceptance",
        "Awaiting delivery outcome", "Delivery confirmed by phone", "Delivery unknown",
        "Delivery failed", "Cancelled on device", "Device unavailable").getOrElse(status.state) { "Unknown" }
}

private fun trigger(value: Int) = listOf("Unknown", "Suspected crash", "Manual SOS",
    "Detector stopped after impact", "Restored incident").getOrElse(value) { "Unknown" }

private fun flightPhase(value: Int?) = listOf("Unknown", "Ground", "Takeoff", "Airborne", "Approach",
    "Landing", "Landed").getOrNull(value ?: -1) ?: "Unavailable"

private fun navigationState(value: Int?) = listOf("Uninitialized", "Initializing", "Tracking",
    "Degraded", "Recovering", "Unavailable", "Numerical fault").getOrNull(value ?: -1) ?: "Unavailable"

private fun warnings(value: Int?): String {
    if (value == null) return "Unavailable"
    val labels = listOf("Excessive descent", "Low ground clearance", "Rapid ground closure", "Hazardous approach",
        "Terrain clearance unavailable", "Navigation degraded")
    return labels.filterIndexed { index, _ -> value and (1 shl index) != 0 }
        .joinToString().ifBlank { "None reported" }
}
