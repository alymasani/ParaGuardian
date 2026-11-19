package org.paraguardian.companion

/** In-process peripheral. Both directions pass through the same framing as the real transport. */
class Simulator(
    private val clock: () -> Long,
    private val packet: (Peer, ByteArray) -> Unit
) {
    companion object { const val ATT = 20 }
    val peer = Peer("simulation-${java.util.UUID.randomUUID()}", Route.SIMULATION, true)
    var connected = false
        private set
    var dropReports = false
    var corruptNext = false
    var missingGnss = true
    var lastReport = 0
        private set

    private var timeOffset = 0L
    private var sequence = 0L
    private var incoming = Protocol.Reassembly()
    private var incident = 0L
    private var state = 0
    private var trigger = 0
    private var deadline = 0L
    private var event = 0L
    private var attempts = 0
    private var lastSend = 0L
    private var acceptedAt = 0L
    private var revision = 1
    private var cycle = 1
    private var lastStatus = 0L
    private var flight = false

    private fun now() = clock() + timeOffset

    fun connect() {
        if (connected) return
        connected = true
        incoming = Protocol.Reassembly()
        sequence = 0
        if (state == 7 && attempts >= 8 && cycle < 3) {
            cycle++
            attempts = 0
            revision++
            state = 2
        }
        if (state == 3) state = 2
        tick()
        status()
    }

    fun disconnect() {
        connected = false
    }

    fun flight() {
        flight = true
        status()
    }

    fun crash() {
        if (state != 0 && state != 8) return
        start(1, 30_000)
    }

    fun advance(ms: Long) {
        timeOffset += ms
        tick()
        status()
    }

    fun resend() {
        if (state in 2..7 && connected) sendSos()
    }

    fun receive(bytes: ByteArray) {
        if (!connected) return
        val message = incoming.push(bytes, now()) ?: return
        when (message.kind) {
            Protocol.COMMAND -> {
                when (message.payload.singleOrNull()?.toInt()) {
                    1 -> if (message.incident == 0L || message.incident == incident) {
                        if (state == 1) {
                            deadline = now() - 1
                            tick()
                        } else if (state == 0 || state == 8) start(2, 5_000)
                    }
                    2 -> if (message.incident == incident && state == 1 && now() <= deadline) {
                        state = 8
                        status()
                    }
                    4 -> status()
                }
            }
            Protocol.REPORT -> {
                if (dropReports || message.incident != incident || message.payload.size != 3) return
                val data = Protocol.wrap(message.payload)
                if ((data.getShort(1).toInt() and 65535) != revision) return
                val report = data.get(0).toInt()
                lastReport = report
                when (report) {
                    Protocol.PHONE_ACCEPTED -> if (state == 2 || state == 3 || (state == 7 && attempts >= 8)) {
                        state = 4
                        acceptedAt = now()
                    }
                    Protocol.DELIVERY_CONFIRMED -> if (state in 3..7) state = 5
                    Protocol.DELIVERY_FAILED -> if (state == 4) state = 7
                    Protocol.DELIVERY_UNKNOWN -> if (state == 4) state = 6
                }
                status()
            }
        }
    }

    fun tick() {
        val time = now()
        if (state == 1 && time > deadline) state = 2
        if (connected && (state == 2 || (state == 3 && time - lastSend >= 12_000))) {
            if (attempts >= 8) state = 7
            else sendSos()
        }
        if (state == 4 && time - acceptedAt >= 120_000) state = 6
        if (connected && time - lastStatus >= 500) status()
    }

    private fun start(type: Int, countdown: Long) {
        incident++
        revision = 1
        attempts = 0
        cycle = 1
        lastReport = 0
        trigger = type
        event = now()
        deadline = event + countdown
        state = 1
        status()
    }

    private fun sendSos() {
        state = 3
        attempts++
        val time = now()
        lastSend = time
        val raw = Protocol.bytes(64)
        raw.put(0, 1).put(1, 1).put(2, trigger.toByte()).put(3, 2)
        raw.putInt(4, incident.toInt()).putShort(8, revision.toShort())
        raw.putInt(12, time.toInt()).putInt(16, event.toInt())
        if (missingGnss) {
            raw.putShort(10, (1 shl 8).toShort())
            raw.putInt(36, -1)
        } else {
            raw.putShort(10, (1 or (1 shl 8)).toShort())
            raw.putInt(20, 513000000).putInt(24, -1153000000)
            raw.putShort(32, 80).put(34, 3).put(35, 8).putInt(36, 1_000)
        }
        raw.putInt(60, Protocol.crc(raw.array(), 60).toInt())
        emit(Protocol.SOS, raw.array())
    }

    private fun status() {
        lastStatus = now()
        val raw = Protocol.bytes(40)
        raw.put(0, state.toByte()).put(1, trigger.toByte())
        raw.put(2, if (state == 5) 6 else if (state in 4..7 && lastReport != 0) 4 else 0)
        raw.put(3, if (state == 7) (if (attempts >= 8) 1 else 2) else 0)
        raw.put(4, attempts.coerceAtMost(255).toByte()).put(5, cycle.toByte())
        raw.putInt(8, if (state == 1) (deadline - now()).coerceAtLeast(0).toInt() else 0)
        if (flight) {
            raw.put(21, 3).put(22, 2).put(24, 7)
            raw.putInt(28, 1830).putInt(32, -112)
        }
        emit(Protocol.STATUS, raw.array())
    }

    private fun emit(kind: Int, raw: ByteArray) {
        if (!connected) return
        val fragments = Protocol.fragments(Protocol.Frame(kind, ++sequence, incident, raw), ATT)
        if (kind == Protocol.SOS && corruptNext) {
            corruptNext = false
            val last = fragments.last()
            last[last.lastIndex] = (last.last().toInt() xor 1).toByte()
        }
        fragments.forEach { packet(peer, it) }
    }
}
