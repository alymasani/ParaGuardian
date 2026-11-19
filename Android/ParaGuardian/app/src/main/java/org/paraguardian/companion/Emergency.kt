package org.paraguardian.companion

enum class Stage(val label: String) {
    STORED("Stored on phone"), READY("Waiting for pilot to send SMS"),
    ATTEMPTING("Send attempt started"), PROVIDER_ACCEPTED("Provider accepted"),
    DELIVERED("Delivery reported"), FAILED("Delivery failed"),
    UNKNOWN("Delivery unknown"), CANCELLED("Cancelled before send")
}

enum class Route { SIMULATION, SMS }

data class Peer(val id: String, val route: Route, val verified: Boolean)
data class Key(val device: String, val incident: Long)
data class Record(
    val key: Key, val revision: Int, val trigger: Int, val received: Long, val eventTime: Long,
    val route: Route, val position: Position?, val raw: ByteArray?, val stage: Stage,
    val attempted: Boolean = false, val cancelledNotice: Boolean = false,
    val composerTargets: Set<String> = emptySet()
) {
    fun reports(): List<Int> {
        if (cancelledNotice) return listOf(Protocol.CANCEL_ACCEPTED)
        return buildList {
            add(Protocol.PHONE_ACCEPTED)
            if (attempted) add(Protocol.SEND_ATTEMPTED)
            when (stage) {
                Stage.DELIVERED -> add(Protocol.DELIVERY_CONFIRMED)
                Stage.FAILED -> add(Protocol.DELIVERY_FAILED)
                Stage.UNKNOWN -> add(Protocol.DELIVERY_UNKNOWN)
                else -> Unit
            }
        }
    }
}

data class Contact(val name: String, val number: String, val enabled: Boolean, val consent: Boolean) {
    val valid get() = name.isNotBlank() && number.matches(Regex("\\+[1-9][0-9]{6,14}"))
    val eligible get() = enabled && consent && valid
}

interface IncidentStore {
    fun get(key: Key): Record?
    fun put(record: Record)
    fun recoverable(): List<Record>
}

interface EmergencyDeliveryService {
    fun send(record: Record): Stage
}

class MockDelivery : EmergencyDeliveryService {
    var result = Stage.DELIVERED
    var network = true
    var attempts = 0
        private set

    override fun send(record: Record): Stage {
        check(record.route == Route.SIMULATION)
        attempts++
        return if (network) result else Stage.FAILED
    }
}

/** All operations run serially on one caller-owned dispatcher, including durable commits. */
class EmergencyEngine(private val store: IncidentStore, private val mock: EmergencyDeliveryService) {
    fun receive(peer: Peer, frame: Protocol.Frame, now: Long, phone: Position? = null): Record? {
        if (!peer.verified || peer.id.isBlank()) return null
        val request = Protocol.sos(frame) ?: return null
        val key = Key(peer.id, request.incident)
        val previous = store.get(key)
        if (previous != null && previous.route != peer.route) return null
        if (previous == null || request.revision > previous.revision) {
            val eventAge = (request.createdUptime - request.eventUptime) and 0xffff_ffffL
            val fresh = Record(
                key, request.revision, request.trigger, now,
                now - eventAge.takeIf { it <= 7L * 24 * 60 * 60 * 1000 }.orZero(),
                peer.route, selectPosition(request.location?.let { it.copy(capturedEpochMs = now - it.ageMs) }, phone),
                frame.payload, Stage.STORED
            )
            val updated = if (request.kind == 2) {
                val base = previous ?: fresh
                base.copy(
                    revision = request.revision, cancelledNotice = true,
                    stage = if (base.attempted || base.composerTargets.isNotEmpty() || base.stage == Stage.UNKNOWN)
                        base.stage else Stage.CANCELLED
                )
            } else if (previous != null) {
                if (previous.cancelledNotice) return null
                previous.copy(revision = request.revision, position = selectPosition(
                    previous.position?.let {
                        val currentAge = it.capturedEpochMs?.let { stamp -> (now - stamp).coerceAtLeast(0) }
                            ?: (it.ageMs + (now - previous.received).coerceAtLeast(0))
                        it.copy(ageMs = currentAge, stale = it.stale || currentAge > 60_000)
                    }, fresh.position
                ))
            } else fresh
            store.put(updated)
        }
        val accepted = store.get(key) ?: return null
        if (accepted.revision != request.revision || accepted.cancelledNotice != (request.kind == 2)) return null
        return accepted
    }

    fun proceed(record: Record): Record {
        val latest = store.get(record.key) ?: return record
        if (latest.stage != Stage.STORED) return latest
        if (latest.route == Route.SMS) return change(latest.copy(stage = Stage.READY))
        val attempt = change(latest.copy(stage = Stage.ATTEMPTING, attempted = true))
        val result = try {
            mock.send(attempt)
        } catch (_: Exception) {
            Stage.UNKNOWN
        }
        require(result in setOf(Stage.PROVIDER_ACCEPTED, Stage.DELIVERED, Stage.FAILED, Stage.UNKNOWN))
        return change(attempt.copy(stage = result))
    }

    fun recover() {
        for (record in store.recoverable()) {
            when (record.stage) {
                Stage.STORED -> proceed(record)
                Stage.ATTEMPTING -> change(record.copy(stage = Stage.UNKNOWN))
                else -> Unit
            }
        }
    }

    fun preparePhoneSos(now: Long, incident: Long, position: Position?): Record {
        val record = Record(Key("phone", incident), 1, 2, now, now, Route.SMS, position, null, Stage.READY)
        check(store.get(record.key) == null)
        return change(record)
    }

    fun composerOpened(key: Key, target: String): Record? {
        val record = store.get(key) ?: return null
        if (record.route != Route.SMS || record.cancelledNotice || target in record.composerTargets ||
            record.composerTargets.size >= 5 || (record.stage != Stage.READY &&
                (record.stage != Stage.UNKNOWN || record.composerTargets.isEmpty()))
        ) return null
        // Persist before launching a separate app. A kill between these operations leaves unknown.
        return change(record.copy(stage = Stage.UNKNOWN, composerTargets = record.composerTargets + target))
    }

    fun composerUnavailable(key: Key, target: String): Record? {
        val record = store.get(key) ?: return null
        if (record.route != Route.SMS || record.stage != Stage.UNKNOWN || target !in record.composerTargets) return null
        val targets = record.composerTargets - target
        return change(record.copy(stage = if (targets.isEmpty()) Stage.READY else Stage.UNKNOWN, composerTargets = targets))
    }

    fun addPhonePosition(key: Key, position: Position): Record? {
        val record = store.get(key) ?: return null
        if (record.route != Route.SMS || record.cancelledNotice || record.stage != Stage.READY) return null
        return change(record.copy(position = selectPosition(record.position, position)))
    }

    private fun change(record: Record): Record {
        store.put(record)
        return record
    }
}

private fun Long?.orZero() = this ?: 0L

fun emergencyMessage(record: Record, additional: String, now: Long): String = buildString {
    if (record.route == Route.SIMULATION) appendLine("SIMULATION ONLY")
    appendLine("ParaGuardian SOS #${record.key.incident} from ${record.key.device}")
    appendLine("Incident (estimated UTC) ${java.time.Instant.ofEpochMilli(record.eventTime)}")
    appendLine(if (record.trigger == 2) "Manual SOS." else "Possible crash. Automated detection is unverified.")
    val position = record.position
    if (position == null) appendLine("Location unavailable.")
    else {
        val age = position.capturedEpochMs?.let { (now - it).coerceAtLeast(0) }
            ?: (position.ageMs + (now - record.received).coerceAtLeast(0))
        val label = if (position.stale || age > 60_000) "Last known" else "Recent"
        val coordinates = "${position.lat},${position.lon}"
        appendLine("$label ${position.source}, ${age / 1000}s old.")
        position.accuracyM?.let { appendLine("Reported accuracy ${it.toInt()} m.") }
        appendLine("https://maps.google.com/?q=$coordinates")
    }
    appendLine("Message prepared ${java.time.Instant.ofEpochMilli(now)}")
    if (additional.isNotBlank()) append(additional)
}

fun contactTarget(number: String): String = java.security.MessageDigest.getInstance("SHA-256")
    .digest(number.toByteArray(Charsets.UTF_8)).joinToString("") { "%02x".format(it.toInt() and 255) }
