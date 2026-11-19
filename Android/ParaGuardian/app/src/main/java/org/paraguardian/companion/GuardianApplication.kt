package org.paraguardian.companion

import android.app.Application
import android.content.Context
import android.os.SystemClock
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

class GuardianApplication : Application() {
    lateinit var guardian: Guardian
        private set

    override fun onCreate() {
        super.onCreate()
        guardian = Guardian(this)
    }
}

data class ViewState(
    val simulation: Boolean = true, val simulatedConnected: Boolean = false,
    val link: LinkState = LinkState.INACTIVE, val monitoring: Boolean = false,
    val lastConnected: Long? = null, val status: Protocol.DeviceStatus? = null,
    val statusAt: Long = 0, val records: List<Record> = emptyList(),
    val timeline: Map<Key, List<IncidentDatabase.Timeline>> = emptyMap(),
    val events: List<String> = emptyList(), val message: String = "",
    val address: String? = null, val contacts: List<Contact> = emptyList(), val template: String = "",
    val safetyNotifications: Boolean = true, val storageReady: Boolean = false
)

class Guardian(private val context: Context) {
    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.Main.immediate)
    private val disk = Dispatchers.IO.limitedParallelism(1)
    private val database = IncidentDatabase(context)
    private val settings = Settings(context)
    private val mutable = MutableStateFlow(ViewState())
    val state = mutable.asStateFlow()
    private val mock = MockDelivery()
    private val engine = EmergencyEngine(database, mock)
    private val notifications = GuardianNotifications(context)
    private var incoming = Protocol.Reassembly()
    private var tx = 0L
    private var generation = 0L
    private var reportDelay = false
    private var paused = false
    private var activityVisible = false
    private var locationRequestedFor: Key? = null
    private val delayed = ArrayDeque<List<Protocol.Frame>>()
    private var lastNotification: Triple<Key, Stage, Boolean>? = null
    private var cachedPhone: Pair<Position, Long>? = null

    private data class Packet(val generation: Long, val peer: Peer, val bytes: ByteArray)
    private val packets = Channel<Packet>(128)
    private var simulator = Simulator(SystemClock::elapsedRealtime, ::enqueue)
    private val physical = BluetoothLink(context, PhysicalTrust { null }, ::linkChanged, ::enqueue)

    init {
        scope.launch {
            guarded("Local storage unavailable") {
                val initial = withContext(disk) {
                    database.redact(System.currentTimeMillis(), false)
                    engine.recover()
                    ViewState(settings.simulation, address = settings.address, contacts = settings.contacts(),
                        records = database.visible(), timeline = database.timeline(), template = settings.template,
                        safetyNotifications = settings.safetyNotifications, storageReady = true,
                        lastConnected = settings.lastPhysicalConnection.takeIf { !settings.simulation && it != 0L })
                }
                mutable.value = initial
            }
            for (packet in packets) {
                if (paused || packet.generation != generation) continue
                guarded("Incoming SOS could not be stored or recovered") { handle(packet) }
            }
        }
        scope.launch {
            var lastRedaction = SystemClock.elapsedRealtime()
            while (true) {
                delay(250)
                if (!paused) simulator.tick()
                val elapsed = SystemClock.elapsedRealtime()
                if (state.value.storageReady && elapsed - lastRedaction >= 60 * 60 * 1000L) {
                    lastRedaction = elapsed
                    guarded("History cleanup failed") {
                        withContext(disk) { database.redact(System.currentTimeMillis(), false) }
                        refresh()
                    }
                }
            }
        }
    }

    private fun enqueue(peer: Peer, bytes: ByteArray) {
        if (!packets.trySend(Packet(generation, peer, bytes)).isSuccess) log("Packet queue full")
    }

    private suspend fun handle(packet: Packet) {
        val sim = state.value.simulation
        if (!packet.peer.verified || sim != (packet.peer.route == Route.SIMULATION)) return
        val before = incoming.rejected
        val frame = incoming.push(packet.bytes, SystemClock.elapsedRealtime())
        if (incoming.rejected != before) log("Rejected malformed or incomplete frame")
        frame ?: return
        if (frame.kind == Protocol.STATUS) {
            val status = Protocol.status(frame) ?: return log("Rejected invalid status")
            val previous = state.value.status
            mutable.update { it.copy(status = status, statusAt = SystemClock.elapsedRealtime()) }
            if (previous?.state != status.state || previous?.incident != status.incident) {
                if (status.state == 1) notifyEmergency("Suspected emergency", "Device countdown active", sim)
                if (status.state == 8) notifyEmergency("Device cancelled SOS", "Review any earlier send attempts", sim)
            }
            if (status.warnings != null && status.warnings != 0 && previous?.warnings != status.warnings &&
                state.value.safetyNotifications
            ) notifyEmergency("Flight safety warning", "Open ParaGuardian for device status", sim)
            return
        }
        if (frame.kind != Protocol.SOS || !state.value.storageReady) return
        val now = System.currentTimeMillis()
        val phone = cachedPhone?.takeIf { !sim }?.let { (value, recorded) ->
            val age = value.ageMs + (now - recorded).coerceAtLeast(0)
            value.copy(ageMs = age, stale = value.stale || age > 60_000, acquiredForIncident = false)
        }
        val accepted = withContext(disk) { engine.receive(packet.peer, frame, now, phone) } ?: return
        if (packet.generation == generation) reports(accepted, false)
        val completed = withContext(disk) { engine.proceed(accepted) }
        if (packet.generation == generation && completed != accepted) reports(completed, true)
        refresh()
        if (!sim && activityVisible && completed.stage == Stage.READY &&
            completed.position?.stale != false && locationRequestedFor != completed.key) {
            locationRequestedFor = completed.key
            perform {
                val fix = visiblePhoneLocation(context, true)
                if (fix != null) {
                    withContext(disk) { engine.addPhonePosition(completed.key, fix) }
                    refresh()
                }
            }
        }
        log(if (completed == accepted) "Durable incident state replayed" else "Incident stored and routed")
        val notification = Triple(completed.key, completed.stage, completed.cancelledNotice)
        if (lastNotification != notification) {
            lastNotification = notification
            notifyEmergency(if (completed.cancelledNotice) "Cancellation recorded" else "SOS · ${completed.stage.label}",
                "Open emergency history for the current incident", sim)
        }
    }

    private fun reports(record: Record, skipAccepted: Boolean) {
        val codes = record.reports().filterNot { skipAccepted && it == Protocol.PHONE_ACCEPTED }
        if (codes.isEmpty()) return
        val frames = codes.map { Protocol.report(0, record.key.incident, record.revision, it) }
        if (state.value.simulation && reportDelay) {
            if (delayed.size < 16) delayed.addLast(frames) else log("Delayed report buffer full")
        } else if (!send(frames)) log("Report write unavailable")
    }

    private fun send(frames: List<Protocol.Frame>): Boolean {
        val simulation = state.value.simulation
        if (simulation && !simulator.connected) return false
        val att = if (simulation) Simulator.ATT else physical.attPayload
        val fragments = frames.flatMap { Protocol.fragments(it.copy(sequence = ++tx), att) }
        return if (simulation) {
            fragments.forEach(simulator::receive)
            true
        } else physical.send(fragments)
    }

    fun command(type: Int, incident: Long = 0) {
        val s = state.value
        if (!s.simulatedConnected && s.link != LinkState.READY) {
            banner("An authenticated connection is required for device commands.")
            return
        }
        if (!send(listOf(Protocol.Frame(Protocol.COMMAND, 0, incident, byteArrayOf(type.toByte()))))) {
            banner("Command could not be queued.")
        } else banner("Command queued. Check device status for the result.")
    }

    private fun resetSession() {
        generation++
        incoming = Protocol.Reassembly()
        tx = 0
        delayed.clear()
        mutable.update { it.copy(status = null, statusAt = 0) }
    }

    fun mode(simulation: Boolean) = perform {
        if (state.value.monitoring) return@perform banner("Stop physical monitoring before changing mode.")
        simulator.disconnect()
        resetSession()
        val lastConnected = withContext(disk) {
            settings.simulation = simulation
            settings.lastPhysicalConnection.takeIf { !simulation && it != 0L }
        }
        mutable.update { it.copy(simulation = simulation, simulatedConnected = false, lastConnected = lastConnected) }
    }

    fun simulationConnect(connect: Boolean) {
        if (!state.value.simulation || simulator.connected == connect) return
        resetSession()
        mutable.update { it.copy(simulatedConnected = connect,
            lastConnected = if (connect) System.currentTimeMillis() else it.lastConnected) }
        if (connect) simulator.connect() else simulator.disconnect()
        log(if (connect) "Simulation connected" else "Simulation disconnected")
    }

    fun simulationAction(action: String) {
        if (!state.value.simulation) return
        when (action) {
            "Flight" -> simulator.flight()
            "Suspect crash" -> simulator.crash()
            "Advance 31 seconds" -> simulator.advance(31_000)
            "Repeat SOS" -> simulator.resend()
            "Corrupt next SOS" -> { simulator.corruptNext = true; banner("Next SOS will be corrupted.") }
            "Toggle GNSS" -> {
                simulator.missingGnss = !simulator.missingGnss
                banner(if (simulator.missingGnss) "Simulated GNSS unavailable." else "Simulated GNSS available.")
            }
            "Toggle report loss" -> { simulator.dropReports = !simulator.dropReports; banner("Drop reports · ${simulator.dropReports}") }
            "Toggle report delay" -> {
                reportDelay = !reportDelay
                if (!reportDelay) while (delayed.isNotEmpty()) {
                    if (!send(delayed.removeFirst())) log("Delayed report could not be written")
                }
                banner("Delay reports · $reportDelay")
            }
            "Toggle network" -> { mock.network = !mock.network; banner("Simulated phone network · ${mock.network}") }
            "Simulate provider accepted" -> {
                mock.result = Stage.PROVIDER_ACCEPTED
                banner("Mock provider will accept the request without a delivery report.")
            }
            "Simulate delivered" -> { mock.result = Stage.DELIVERED; banner("Mock provider will report delivery.") }
            "Simulate failed" -> { mock.result = Stage.FAILED; banner("Mock provider will report failure.") }
            "Simulate unknown" -> { mock.result = Stage.UNKNOWN; banner("Mock provider will report an unknown outcome.") }
            "Restart incident engine" -> perform {
                paused = true
                try {
                    withContext(disk) { database.close(); engine.recover() }
                    refresh()
                } finally { paused = false }
                banner("Incident data reopened from SQLite.")
            }
            "New simulated device" -> {
                simulationConnect(false)
                simulator = Simulator(SystemClock::elapsedRealtime, ::enqueue)
                banner("A new simulated device identity is ready to connect.")
            }
        }
    }

    fun monitor(address: String): Boolean {
        if (state.value.simulation || !state.value.storageReady || !context.canConnect()) return false
        mutable.update { it.copy(monitoring = true) }
        physical.start(address)
        return true
    }

    fun stopMonitor() {
        physical.stop()
        mutable.update { it.copy(monitoring = false) }
    }

    private fun linkChanged(link: LinkState) {
        if (link == state.value.link) return
        resetSession()
        val connected = link == LinkState.READY
        mutable.update { it.copy(link = link, lastConnected = if (connected) System.currentTimeMillis() else it.lastConnected) }
        log(link.label)
        if (connected) perform { withContext(disk) { settings.lastPhysicalConnection = System.currentTimeMillis() } }
        if (connected || link == LinkState.BACKOFF || link == LinkState.DISABLED || link == LinkState.PERMISSION) {
            notifyEmergency("Device monitoring", link.label, false, critical = false)
        }
    }

    fun associate(address: String?) = perform {
        withContext(disk) { settings.address = address }
        mutable.update { it.copy(address = address) }
    }

    fun saveContacts(contacts: List<Contact>) = perform {
        withContext(disk) { settings.saveContacts(contacts) }
        mutable.update { it.copy(contacts = contacts) }
    }

    fun template(value: String) = perform {
        withContext(disk) { settings.template = value }
        mutable.update { it.copy(template = value.take(160)) }
    }

    fun safetyNotifications(enabled: Boolean) = perform {
        withContext(disk) { settings.safetyNotifications = enabled }
        mutable.update { it.copy(safetyNotifications = enabled) }
    }

    fun activityVisible(visible: Boolean) { activityVisible = visible }

    fun cachePosition(position: Position?) {
        if (position != null) cachedPhone = position to System.currentTimeMillis()
    }

    fun phoneSos(position: Position?) = perform {
        if (state.value.simulation || state.value.contacts.none { it.eligible }) {
            return@perform banner("Select physical mode and enable a consented SMS contact.")
        }
        val now = System.currentTimeMillis()
        withContext(disk) {
            var id = now
            while (database.get(Key("phone", id)) != null) id++
            engine.preparePhoneSos(now, id, position)
        }
        refresh()
        banner("Draft ready. Choose a contact on the Emergency screen.")
    }

    fun openComposer(key: Key, contact: Contact, launch: (String, String) -> Boolean) = perform {
        if (state.value.simulation || !contact.eligible || contact !in state.value.contacts) return@perform
        val target = contactTarget(contact.number)
        val record = withContext(disk) { engine.composerOpened(key, target) } ?: return@perform
        if (state.value.simulation || !activityVisible || !launch(contact.number,
                emergencyMessage(record, state.value.template, System.currentTimeMillis()))) {
            withContext(disk) { engine.composerUnavailable(key, target) }
            return@perform banner("SMS composer unavailable.")
        }
        reportsForPhysical(record)
        refresh()
        banner("SMS composer opened. Confirm send there. Delivery status is unknown.")
    }

    private fun reportsForPhysical(record: Record) {
        if (record.key.device != "phone" && state.value.link == LinkState.READY) reports(record, true)
    }

    fun clearHistory() = perform {
        withContext(disk) { database.redact(System.currentTimeMillis(), true) }
        refresh()
        banner("Local incident history cleared.")
    }

    private suspend fun refresh() {
        val (records, timeline) = withContext(disk) { database.visible() to database.timeline() }
        mutable.update { it.copy(records = records, timeline = timeline) }
    }

    private fun perform(action: suspend () -> Unit) {
        scope.launch { guarded("Operation failed. Check local storage and configuration.", action) }
    }

    private suspend fun guarded(message: String, action: suspend () -> Unit) {
        try { action() }
        catch (e: kotlinx.coroutines.CancellationException) { throw e }
        catch (_: Exception) { banner(message); log(message) }
    }

    fun banner(message: String) { mutable.update { it.copy(message = message) } }
    private fun log(message: String) { mutable.update { it.copy(events = (listOf(message) + it.events).take(60)) } }

    private fun notifyEmergency(title: String, message: String, sim: Boolean, critical: Boolean = true) {
        val sent = notifications.alert(if (sim) "Simulation · $title" else title, message, critical)
        if (!sent) log("Notification unavailable or permission denied")
    }
}
