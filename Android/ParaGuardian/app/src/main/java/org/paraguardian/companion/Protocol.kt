package org.paraguardian.companion

import java.nio.ByteBuffer
import java.nio.ByteOrder.LITTLE_ENDIAN
import java.util.zip.CRC32

object Protocol {
    const val SOS = 1
    const val REPORT = 2
    const val COMMAND = 3
    const val STATUS = 4
    private const val VERSION = 1
    private const val HEADER = 12
    private const val OVERHEAD = 16
    private const val FRAGMENT = 6
    private const val MAX_FRAME = 112
    private const val MAX_SEQUENCE = 0xffff_ffffL

    const val PHONE_ACCEPTED = 5
    const val SEND_ATTEMPTED = 7
    const val DELIVERY_CONFIRMED = 8
    const val DELIVERY_FAILED = 9
    const val CANCEL_ACCEPTED = 10
    const val DELIVERY_UNKNOWN = 11

    data class Frame(val kind: Int, val sequence: Long, val incident: Long, val payload: ByteArray)

    fun bytes(size: Int): ByteBuffer = ByteBuffer.allocate(size).order(LITTLE_ENDIAN)
    fun wrap(bytes: ByteArray): ByteBuffer = ByteBuffer.wrap(bytes).order(LITTLE_ENDIAN)
    fun ByteBuffer.u8(offset: Int) = get(offset).toInt() and 255
    fun ByteBuffer.u16(offset: Int) = getShort(offset).toInt() and 65535
    fun ByteBuffer.u32(offset: Int) = getInt(offset).toLong() and MAX_SEQUENCE

    fun crc(data: ByteArray, size: Int): Long = CRC32().apply { update(data, 0, size) }.value

    fun encode(frame: Frame): ByteArray {
        require(frame.kind in 1..5 && frame.sequence in 1..MAX_SEQUENCE)
        require(frame.incident in 0..MAX_SEQUENCE && frame.payload.size <= MAX_FRAME - OVERHEAD)
        val buffer = bytes(frame.payload.size + OVERHEAD)
        buffer.put(VERSION.toByte()).put(frame.kind.toByte()).putInt(frame.sequence.toInt())
        buffer.putInt(frame.incident.toInt()).putShort(frame.payload.size.toShort()).put(frame.payload)
        buffer.putInt(crc(buffer.array(), buffer.position()).toInt())
        return buffer.array()
    }

    fun fragments(frame: Frame, att: Int): List<ByteArray> {
        require(att >= 20)
        val data = encode(frame)
        return data.asList().indices.step((att - FRAGMENT).coerceAtMost(MAX_FRAME)).map { offset ->
            val size = minOf(att - FRAGMENT, data.size - offset)
            bytes(size + FRAGMENT).putInt(frame.sequence.toInt()).put(offset.toByte())
                .put(data.size.toByte()).put(data, offset, size).array()
        }
    }

    fun report(sequence: Long, incident: Long, revision: Int, kind: Int) =
        Frame(REPORT, sequence, incident, bytes(3).put(kind.toByte()).putShort(revision.toShort()).array())

    class Reassembly {
        private val buffer = ByteArray(MAX_FRAME)
        private var used = 0
        private var total = 0
        private var sequence = 0L
        private var lastComplete = 0L
        private var started = 0L
        var rejected = 0
            private set

        fun push(packet: ByteArray, now: Long): Frame? {
            if (used != 0 && now - started >= 2_000) {
                used = 0
                rejected++
            }
            if (packet.size !in (FRAGMENT + 1)..(MAX_FRAME + FRAGMENT)) return invalid()
            val header = wrap(packet)
            val id = header.u32(0)
            val offset = header.u8(4)
            val expected = header.u8(5)
            val count = packet.size - FRAGMENT
            if (id == 0L || expected !in OVERHEAD..MAX_FRAME || offset + count > expected) return invalid()
            if (id <= lastComplete) return null
            if (offset == 0 && (used == 0 || id > sequence)) {
                used = 0
                sequence = id
                total = expected
                started = now
            }
            if (id != sequence || total != expected || offset > used) return invalid()
            if (offset < used) {
                if (offset + count > used || (0 until count).any { buffer[offset + it] != packet[FRAGMENT + it] }) {
                    return invalid()
                }
                return null
            }
            packet.copyInto(buffer, offset, FRAGMENT)
            used += count
            if (used != total) return null
            used = 0
            val frame = wrap(buffer)
            val kind = frame.u8(1)
            if (frame.u8(0) != VERSION || kind !in 1..5 || frame.u32(2) != id ||
                frame.u16(10) + OVERHEAD != total || frame.u32(total - 4) != crc(buffer, total - 4)
            ) return invalid()
            lastComplete = id
            return Frame(kind, id, frame.u32(6), buffer.copyOfRange(HEADER, total - 4))
        }

        private fun invalid(): Frame? {
            rejected++
            return null
        }
    }

    data class Sos(
        val incident: Long, val revision: Int, val kind: Int, val trigger: Int, val flags: Int,
        val createdUptime: Long, val eventUptime: Long, val location: Position?
    )

    fun sos(frame: Frame): Sos? {
        val raw = frame.payload
        if (frame.kind != SOS || raw.size != 64) return null
        val b = wrap(raw)
        if (b.u8(0) != 1 || b.u8(1) !in 1..2 || b.u8(2) !in 1..4 || b.u8(3) > 9 ||
            frame.incident == 0L || frame.incident != b.u32(4) || b.u16(8) == 0 ||
            b.u32(60) != crc(raw, 60)
        ) return null
        val flags = b.u16(10)
        val lat = b.getInt(20) / 1e7
        val lon = b.getInt(24) / 1e7
        val age = b.u32(36)
        val location = if (flags and 1 != 0 && lat in -90.0..90.0 && lon in -180.0..180.0 && age != MAX_SEQUENCE) {
            Position(lat, lon, age, b.u16(32) / 10.0, "Device GNSS", flags and 128 != 0 || age > 60_000)
        } else null
        return Sos(frame.incident, b.u16(8), b.u8(1), b.u8(2), flags, b.u32(12), b.u32(16), location)
    }

    data class DeviceStatus(
        val incident: Long, val state: Int, val trigger: Int, val stage: Int, val failure: Int,
        val attempts: Int, val cycles: Int, val countdownMs: Long, val retryMs: Long, val flags: Long,
        val cancelNotice: Int, val phase: Int?, val nav: Int?, val warnings: Int?, val altitude: Double?,
        val speed: Double?, val batteryMv: Int?, val quality: Long
    )

    fun status(frame: Frame): DeviceStatus? {
        if (frame.kind != STATUS || frame.payload.size != 40) return null
        val b = wrap(frame.payload)
        val valid = b.u8(24)
        val safety = valid and 4 != 0
        if (b.u8(0) > 9 || b.u8(1) > 4 || b.u8(2) > 6 || b.u8(3) > 2) return null
        return DeviceStatus(
            frame.incident, b.u8(0), b.u8(1), b.u8(2), b.u8(3), b.u8(4), b.u8(5),
            b.u32(8), b.u32(12), b.u32(16), b.u8(20),
            b.u8(21).takeIf { safety }, b.u8(22).takeIf { safety }, b.u8(23).takeIf { safety },
            (b.getInt(28) / 10.0).takeIf { valid and 1 != 0 },
            (b.getInt(32) / 100.0).takeIf { valid and 2 != 0 }, b.u16(26).takeIf { it != 0 }, b.u32(36)
        )
    }
}

data class Position(
    val lat: Double, val lon: Double, val ageMs: Long, val accuracyM: Double?,
    val source: String, val stale: Boolean, val acquiredForIncident: Boolean = false,
    val capturedEpochMs: Long? = null
)

fun selectPosition(device: Position?, phone: Position?): Position? = listOfNotNull(device, phone)
    .minWithOrNull(compareBy<Position> { it.stale }.thenBy { if (it.stale) it.ageMs else 0 }
        .thenBy { if (it.source == "Device GNSS") 0 else 1 })
