package org.paraguardian.companion

import org.junit.Assert.*
import org.junit.Assume.assumeTrue
import org.junit.Test
import java.io.DataInputStream
import java.io.DataOutputStream
import java.io.File
import java.nio.file.Files
import java.nio.file.StandardCopyOption

class EmergencyTest {
    private val vectors by lazy {
        checkNotNull(javaClass.getResourceAsStream("/ble_v1.txt")).bufferedReader().useLines { lines ->
            lines.associate { it.substringBefore(' ') to it.substringAfter(' ').hexBytes() }
        }
    }
    private val incident = 0x01020304L
    private val peer = Peer("sim", Route.SIMULATION, true)

    @Test fun sharedVectorsMatchBothDirections() {
        val frames = mapOf("alert" to alert(),
            "accepted" to Protocol.report(1, incident, 9, Protocol.PHONE_ACCEPTED),
            "cancel_command" to Protocol.Frame(3, 2, incident, byteArrayOf(2)))
        frames.forEach { (name, frame) ->
            assertArrayEquals(vectors.getValue(name), Protocol.encode(frame))
            Protocol.fragments(frame, 20).forEachIndexed { index, bytes ->
                assertArrayEquals(vectors.getValue("${name}_att20_$index"), bytes)
            }
        }
        val decoded = checkNotNull(Protocol.sos(alert()))
        assertEquals(incident, decoded.incident)
        assertEquals(9, decoded.revision)
        assertEquals(-115.3, decoded.location!!.lon, 1e-7)
    }

    @Test fun mtuRangeDuplicateAndReplay() {
        for (att in listOf(20, 38, 118, 244)) {
            val receiver = Protocol.Reassembly()
            val fragments = Protocol.fragments(alert(), att)
            var frame: Protocol.Frame? = null
            fragments.forEachIndexed { i, bytes ->
                frame = receiver.push(bytes, i.toLong())
                assertNull(receiver.push(bytes, i.toLong()))
            }
            assertEquals(incident, frame?.incident)
            assertEquals(0, receiver.rejected)
        }
    }

    @Test fun malformedMissingOversizedAndVersionDowngrade() {
        val fragments = Protocol.fragments(alert(), 20)
        val receiver = Protocol.Reassembly()
        assertNull(receiver.push(fragments[2], 0))
        assertNull(receiver.push(fragments[0], 10))
        assertNull(receiver.push(fragments[1], 2_011))
        assertNull(Protocol.Reassembly().push(fragments[1], 2_012))
        assertNull(receiver.push(ByteArray(119), 2_013))
        val bad = fragments.map(ByteArray::clone)
        bad.last()[bad.last().lastIndex] = 0
        bad.forEach { assertNull(receiver.push(it, 2_014)) }
        assertTrue(receiver.rejected >= 4)
        val downgrade = fragments.map(ByteArray::clone)
        downgrade.first()[6] = 0
        downgrade.forEach { assertNull(receiver.push(it, 2_015)) }
        assertNotNull(fragments.mapNotNull { receiver.push(it, 2_016) }.single())
    }

    @Test fun rejectsUnknownPeersAndInnerIncidentMismatch() {
        val store = Memory()
        val mock = MockDelivery()
        val engine = EmergencyEngine(store, mock)
        assertNull(engine.receive(peer.copy(verified = false), alert(), 5_000))
        assertNull(engine.receive(peer, alert().copy(incident = 5), 5_000))
        assertEquals(0, store.records().size)
        assertEquals(0, mock.attempts)
    }

    @Test fun persistentAcceptancePrecedesAttemptAndRetryDoesNotResend() {
        val store = Memory()
        val mock = MockDelivery()
        val engine = EmergencyEngine(store, mock)
        val accepted = engine.receive(peer, alert(), 5_000)!!
        assertEquals(Stage.STORED, store.get(accepted.key)!!.stage)
        assertEquals(listOf(Protocol.PHONE_ACCEPTED), accepted.reports())
        val sent = engine.proceed(accepted)
        assertEquals(listOf(5, 7, 8), sent.reports())
        val recreated = EmergencyEngine(store, mock)
        recreated.recover()
        val repeated = recreated.receive(peer, alert(10), 7_000)!!
        recreated.proceed(repeated)
        assertEquals(1, mock.attempts)
        assertEquals(10, store.get(accepted.key)!!.revision)
    }

    @Test fun duplicateIdsAcrossDevicesProduceSeparateRecords() {
        val store = Memory()
        val mock = MockDelivery()
        val engine = EmergencyEngine(store, mock)
        listOf(peer, peer.copy(id = "other")).forEach {
            engine.proceed(engine.receive(it, alert(), 5_000)!!)
        }
        assertEquals(2, mock.attempts)
        assertEquals(2, store.records().size)
    }

    @Test fun cancellationBeforeSendAndLateNoticePreserveTruth() {
        val store = Memory()
        val mock = MockDelivery()
        val engine = EmergencyEngine(store, mock)
        engine.receive(peer, alert(), 5_000)
        val cancelled = engine.receive(peer, alert(10, kind = 2), 6_000)!!
        assertEquals(Stage.CANCELLED, engine.proceed(cancelled).stage)
        assertEquals(listOf(10), cancelled.reports())
        assertNull(engine.receive(peer, alert(11), 7_000))
        val other = peer.copy(id = "other")
        engine.proceed(engine.receive(other, alert(), 5_000)!!)
        val late = engine.receive(other, alert(10, kind = 2), 6_000)!!
        assertEquals(Stage.DELIVERED, late.stage)
        assertTrue(late.cancelledNotice)
        assertEquals(1, mock.attempts)
    }

    @Test fun interruptedAttemptRecoversAsUnknown() {
        val store = Memory()
        val mock = MockDelivery()
        val engine = EmergencyEngine(store, mock)
        val entry = engine.receive(peer, alert(), 5_000)!!
        store.put(entry.copy(stage = Stage.ATTEMPTING, attempted = true))
        EmergencyEngine(store, mock).recover()
        assertEquals(Stage.UNKNOWN, store.get(entry.key)!!.stage)
        assertEquals(listOf(5, 7, 11), store.get(entry.key)!!.reports())
        assertEquals(0, mock.attempts)
    }

    @Test fun unavailableNetworkAndProviderUnknownHaveDistinctReports() {
        for ((network, expected) in listOf(false to Protocol.DELIVERY_FAILED, true to Protocol.DELIVERY_UNKNOWN)) {
            val mock = MockDelivery().apply { this.network = network; result = Stage.UNKNOWN }
            val engine = EmergencyEngine(Memory(), mock)
            assertEquals(listOf(5, 7, expected), engine.proceed(engine.receive(peer, alert(), 5_000)!!).reports())
        }
    }

    @Test fun smsComposerNeverClaimsAnAutomaticSendOrDelivery() {
        val engine = EmergencyEngine(Memory(), MockDelivery())
        val realPeer = Peer("physical-test", Route.SMS, true)
        val draft = engine.proceed(engine.receive(realPeer, alert(), 5_000)!!)
        assertEquals(Stage.READY, draft.stage)
        assertEquals(listOf(5), draft.reports())
        val unknown = engine.composerOpened(draft.key, "contact-a")!!
        assertEquals(listOf(5, 11), unknown.reports())
        assertNull(engine.composerOpened(draft.key, "contact-a"))
        assertEquals(Stage.READY, engine.composerUnavailable(draft.key, "contact-a")!!.stage)
        assertEquals(Stage.READY, engine.preparePhoneSos(7_000, 7, null).stage)
    }

    @Test fun missingAndStaleLocationAreLabelledAndContactsRequireConsent() {
        val result = EmergencyEngine(Memory(), MockDelivery()).receive(peer, alert(noLocation = true), 5_000)!!
        assertNull(result.position)
        assertTrue(emergencyMessage(result, "", 7_000).contains("Location unavailable"))
        val older = EmergencyEngine(Memory(), MockDelivery()).receive(peer, alert(), 5_000)!!
        assertTrue(emergencyMessage(older, "", 120_000).contains("Last known Device GNSS"))
        assertTrue(emergencyMessage(older, "", 120_000).startsWith("SIMULATION ONLY"))
        assertFalse(Contact("Pilot", "+14035550123", true, false).eligible)
        assertFalse(Contact("Pilot", "garbage", true, true).eligible)
        assertTrue(Contact("Pilot", "+14035550123", true, true).eligible)
    }

    @Test fun storageErrorCannotReturnAnAcceptanceOrTriggerProvider() {
        val broken = object : IncidentStore {
            override fun get(key: Key): Record? = null
            override fun put(record: Record) = error("simulated write failure")
            override fun recoverable(): List<Record> = emptyList()
        }
        val mock = MockDelivery()
        assertThrows(IllegalStateException::class.java) {
            EmergencyEngine(broken, mock).receive(peer, alert(), 5_000)
        }
        assertEquals(0, mock.attempts)
    }

    @Test fun simulatorCountdownCancellationAndProtocolStatus() {
        var clock = 100L
        var receiver = Protocol.Reassembly()
        val frames = mutableListOf<Protocol.Frame>()
        val sim = Simulator({ clock }) { _, bytes -> receiver.push(bytes, clock)?.let(frames::add) }
        sim.connect()
        sim.flight()
        sim.crash()
        val status = frames.mapNotNull(Protocol::status).last()
        assertEquals(30_000L, status.countdownMs)
        assertEquals(3, status.phase)
        Protocol.fragments(Protocol.Frame(3, 1, status.incident + 1, byteArrayOf(2)), 20).forEach(sim::receive)
        assertEquals(1, frames.mapNotNull(Protocol::status).last().state)
        Protocol.fragments(Protocol.Frame(3, 2, status.incident, byteArrayOf(2)), 20).forEach(sim::receive)
        assertEquals(8, frames.mapNotNull(Protocol::status).last().state)
        clock += 40_000
        sim.tick()
        assertFalse(frames.any { it.kind == Protocol.SOS })
        sim.disconnect()
        receiver = Protocol.Reassembly()
        sim.connect()
        Protocol.fragments(Protocol.Frame(3, 1, 0, byteArrayOf(1)), 20).forEach(sim::receive)
        sim.advance(6_000)
        assertEquals(2, frames.mapNotNull(Protocol::sos).single().trigger)
    }

    @Test(timeout = 20_000) fun nativeCrashBridgeRoundTripWithLostAckAndDuplicateOnReconnect() {
        val executable = System.getenv("PG_BLE_PEER")
        assumeTrue("Set PG_BLE_PEER to the executable from Tests/build_ble_peer.sh", executable != null)
        check(File(executable!!).canExecute())
        val process = ProcessBuilder(executable).redirectError(ProcessBuilder.Redirect.INHERIT).start()
        val location = Files.createTempDirectory("pg-ble-test").toFile()
        val durableFile = location.resolve("incident")
        try {
            process.inputStream.bufferedReader().use { from ->
                process.outputStream.bufferedWriter().use { to ->
                    fun command(value: String) { to.write(value); to.newLine(); to.flush() }
                    fun readSos(): Protocol.Frame {
                        val reassembly = Protocol.Reassembly()
                        val frames = generateSequence { from.readLine() }.takeWhile { it != "END" }
                            .mapNotNull { reassembly.push(it.hexBytes(), 100) }.toList()
                        return frames.single { it.kind == Protocol.SOS }
                    }
                    assertEquals("FLIGHT COUNTDOWN SOS", from.readLine())
                    val frame = readSos()
                    val store = SingleIncidentDiskStore(durableFile)
                    val mock = MockDelivery()
                    val first = EmergencyEngine(store, mock)
                    val stored = first.receive(peer, frame, 5_000)!!
                    assertNull(stored.position)
                    assertEquals(1, stored.trigger)
                    assertEquals(Stage.STORED, stored.stage)
                    assertEquals(Stage.DELIVERED, first.proceed(stored).stage)
                    // Phone accepted and delivered, but every application acknowledgment was lost.
                    command("state")
                    assertTrue(from.readLine().startsWith("3 "))
                    command("reconnect")
                    val repeated = readSos()
                    val reopened = EmergencyEngine(SingleIncidentDiskStore(durableFile), mock)
                    reopened.recover()
                    val result = reopened.proceed(reopened.receive(peer, repeated, 6_000)!!)
                    assertEquals(stored.key, result.key)
                    assertEquals(1, mock.attempts)
                    result.reports().forEachIndexed { index, stage ->
                        Protocol.fragments(Protocol.report(index.toLong() + 1, result.key.incident, result.revision, stage), 20)
                            .forEach { command(it.hexString()) }
                    }
                    command("state")
                    val final = from.readLine().split(" ").map(String::toInt)
                    assertEquals(5, final[0])
                    assertEquals(stored.key.incident.toInt(), final[1])
                    assertEquals(0x38, final[2] and 0x38)
                }
            }
            assertEquals(0, process.waitFor())
        } finally {
            process.destroyForcibly()
            location.deleteRecursively()
        }
    }

    // Native-peer test uses one no-GNSS incident and reopens it through a fresh store.
    // Android instrumentation exercises the production SQLite implementation separately.
    private class SingleIncidentDiskStore(private val file: File) : IncidentStore {
        override fun get(key: Key): Record? = read()?.takeIf { it.key == key }
        override fun recoverable(): List<Record> = listOfNotNull(read())

        override fun put(record: Record) {
            check(record.position == null && record.route == Route.SIMULATION)
            val raw = checkNotNull(record.raw)
            val candidate = File(file.parentFile, "candidate")
            DataOutputStream(candidate.outputStream()).use {
                it.writeUTF(record.key.device)
                it.writeLong(record.key.incident)
                it.writeInt(record.revision)
                it.writeInt(record.trigger)
                it.writeLong(record.received)
                it.writeLong(record.eventTime)
                it.write(raw)
                it.writeUTF(record.stage.name)
                it.writeBoolean(record.attempted)
                it.writeBoolean(record.cancelledNotice)
            }
            Files.move(candidate.toPath(), file.toPath(), StandardCopyOption.ATOMIC_MOVE,
                StandardCopyOption.REPLACE_EXISTING)
        }

        private fun read(): Record? {
            if (!file.exists()) return null
            return DataInputStream(file.inputStream()).use {
                val key = Key(it.readUTF(), it.readLong())
                Record(key, it.readInt(), it.readInt(), it.readLong(), it.readLong(),
                    Route.SIMULATION, null, ByteArray(64).apply(it::readFully), Stage.valueOf(it.readUTF()),
                    it.readBoolean(), it.readBoolean())
            }
        }
    }

    private fun alert(revision: Int = 9, kind: Int = 1, noLocation: Boolean = false): Protocol.Frame {
        val raw = vectors.getValue("sos").clone()
        val b = Protocol.wrap(raw)
        b.put(1, kind.toByte()).putShort(8, revision.toShort())
        if (noLocation) b.putShort(10, 0).putInt(36, -1)
        b.putInt(60, Protocol.crc(raw, 60).toInt())
        return Protocol.Frame(Protocol.SOS, 0x11223344, incident, raw)
    }

    private class Memory : IncidentStore {
        private val records = mutableMapOf<Key, Record>()
        override fun get(key: Key) = records[key]
        override fun put(record: Record) { records[record.key] = record }
        override fun recoverable() = records.values.filter { it.stage == Stage.STORED || it.stage == Stage.ATTEMPTING }
        fun records() = records.values.toList()
    }
}

private fun String.hexBytes() = chunked(2).map { it.toInt(16).toByte() }.toByteArray()
private fun ByteArray.hexString() = joinToString("") { "%02x".format(it.toInt() and 255) }
