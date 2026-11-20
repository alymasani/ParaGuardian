package org.paraguardian.companion

import org.junit.Assert.*
import org.junit.Assume.assumeTrue
import org.junit.Test
import java.io.File
import java.util.concurrent.TimeUnit

/**
 * Milestone 8: internet delivery engine, idempotent outbox, honest stage mapping, monitoring state,
 * and (when python3 is available) the REAL backend over loopback HTTP with its fake provider.
 */
class DeliveryTest {
    private val vectors by lazy {
        checkNotNull(javaClass.getResourceAsStream("/ble_v1.txt")).bufferedReader().useLines { lines ->
            lines.associate { it.substringBefore(' ') to it.substringAfter(' ').chunked(2).map { h -> h.toInt(16).toByte() }.toByteArray() }
        }
    }
    private val incident = 0x01020304L
    private val internetPeer = Peer("pg-dev-1", Route.INTERNET, true)

    private fun sos(revision: Int, kind: Int = 1, flags: Int? = null): Protocol.Frame {
        val raw = vectors.getValue("sos").clone()
        val b = Protocol.wrap(raw)
        b.put(1, kind.toByte()).putShort(8, revision.toShort())
        if (flags != null) b.putShort(10, flags.toShort())
        b.putInt(60, Protocol.crc(raw, 60).toInt())
        return Protocol.Frame(Protocol.SOS, 0x11223344, incident, raw)
    }

    private class Memory : IncidentStore, OutboxStore {
        val records = mutableMapOf<Key, Record>()
        val outbox = linkedMapOf<Key, OutboxItem>()
        override fun get(key: Key) = records[key]
        override fun put(record: Record) { records[record.key] = record }
        override fun recoverable() = records.values.filter {
            it.stage in setOf(Stage.STORED, Stage.ATTEMPTING, Stage.SUBMISSION_PENDING, Stage.SERVER_ACCEPTED)
        }
        override fun outbox(key: Key) = outbox[key]
        override fun putOutbox(item: OutboxItem) { outbox[item.key] = item }
        override fun outboxItems() = outbox.values.toList()
    }

    /** Scripted server: answers in order; records every request. */
    private class Script(vararg answers: HttpResult) : HttpTransport {
        val queue = ArrayDeque(answers.toList())
        val requests = mutableListOf<Triple<String, String, String>>()
        override fun exchange(method: String, url: String, headers: Map<String, String>, body: ByteArray?): HttpResult {
            assertNotNull(headers["X-PG-Signature"])
            requests += Triple(method, url, body?.toString(Charsets.UTF_8).orEmpty())
            return queue.removeFirstOrNull() ?: HttpResult(HttpResult.NETWORK, "")
        }
    }

    private val cfg = BackendConfig("https://sos.example.invalid", "pg-dev-1", ByteArray(32) { it.toByte() })

    private fun rig(script: Script): Triple<Memory, EmergencyEngine, RemoteDelivery> {
        val mem = Memory()
        val remote = RemoteDelivery({ cfg }, script, mem, nonce = { "nonce0123456789abcdef" })
        var now = 1_000L
        val queue = object : RemoteQueue {
            override fun handles(record: Record) = record.route == Route.INTERNET
            override fun enqueue(record: Record) { remote.enqueue(record, false, now++) }
        }
        return Triple(mem, EmergencyEngine(mem, MockDelivery(), queue), remote)
    }

    @Test fun signerMatchesTheBackendReferenceVector() {
        // Computed by Backend/paraguardian_sos/auth.py sign() with the same inputs.
        assertEquals("e7915d6691bdefc7374602600cbcb2826c75eef6f39469d21167906817ad4dc6",
            RequestSigner.sign(ByteArray(32) { it.toByte() }, "POST", "/v1/incidents", 1_800_000_000L,
                "nonce0123456789abcdef", "{\"incidentKey\":\"7\"}".toByteArray()))
    }

    @Test fun offlineSubmissionIsDurableRetriedAndNeverReportedDelivered() {
        val script = Script(HttpResult(HttpResult.NETWORK, ""), HttpResult(503, ""),
            HttpResult(202, """{"overall":"pending","result":"created"}"""))
        val (mem, engine, remote) = rig(script)
        val accepted = checkNotNull(engine.receive(internetPeer, sos(1), 1_000L))
        val pending = engine.proceed(accepted)
        assertEquals(Stage.SUBMISSION_PENDING, pending.stage)
        assertEquals(OutboxState.PENDING, mem.outbox.values.single().state)   // durable before any network call
        var updates = remote.runOnce(2_000L)
        assertEquals(Stage.SUBMISSION_PENDING, updates.single().stage)
        assertEquals("Internet unavailable", remote.lastNetworkProblem)
        assertTrue(remote.runOnce(2_001L).isEmpty())                            // backing off: no hammering
        updates = remote.runOnce(2_000L + 5_000L)
        assertEquals("Emergency server busy (HTTP 503)", remote.lastNetworkProblem)
        updates = remote.runOnce(30_000L)
        assertEquals(Stage.SERVER_ACCEPTED, updates.single().stage)
        val record = checkNotNull(engine.applyRemote(updates.single()))
        assertEquals(Stage.SERVER_ACCEPTED, record.stage)
        assertNotEquals(Stage.DELIVERED, record.stage)                          // a server 202 is not delivery
        assertTrue(record.attempted)
        assertEquals(3, script.requests.size)
        assertTrue(script.requests.all { it.third == script.requests[0].third })   // identical (idempotent) body
    }

    @Test fun pollingReconcilesProviderAcceptanceDeliveryAndNeverDowngrades() {
        val script = Script(HttpResult(202, """{"overall":"pending"}"""),
            HttpResult(200, """{"overall":"provider_accepted"}"""), HttpResult(200, """{"overall":"delivered"}"""))
        val (mem, engine, remote) = rig(script)
        engine.proceed(checkNotNull(engine.receive(internetPeer, sos(1), 1_000L)))
        remote.runOnce(2_000L).forEach { engine.applyRemote(it) }
        remote.runOnce(2_000L + RemoteDelivery.POLL_MS).forEach { engine.applyRemote(it) }
        assertEquals(Stage.PROVIDER_ACCEPTED, mem.records.values.single().stage)
        remote.runOnce(2_000L + 2 * RemoteDelivery.POLL_MS).forEach { engine.applyRemote(it) }
        assertEquals(Stage.DELIVERED, mem.records.values.single().stage)
        assertEquals(OutboxState.DONE, mem.outbox.values.single().state)
        assertNull(engine.applyRemote(RemoteUpdate(mem.records.keys.single(), Stage.SUBMISSION_PENDING, "late")))
        assertEquals(listOf(Protocol.PHONE_ACCEPTED, Protocol.SEND_ATTEMPTED, Protocol.DELIVERY_CONFIRMED),
            mem.records.values.single().reports())
    }

    @Test fun duplicatesRevisionsCancellationAndRejectedCredentials() {
        val script = Script(HttpResult(202, """{"overall":"pending"}"""), HttpResult(200, """{"overall":"pending"}"""),
            HttpResult(401, """{"error":"unauthorized"}"""))
        val (mem, engine, remote) = rig(script)
        engine.proceed(checkNotNull(engine.receive(internetPeer, sos(1), 1_000L)))
        engine.receive(internetPeer, sos(1), 1_100L)                // MCU resend of the same revision
        assertEquals(1, mem.outbox.values.single().revision)
        remote.runOnce(2_000L).forEach { engine.applyRemote(it) }
        engine.receive(internetPeer, sos(2, kind = 2), 3_000L)       // cancellation after the server has it
        val item = mem.outbox.values.single()
        assertEquals(Pair("cancel", OutboxState.PENDING), Pair(item.kind, item.state))
        assertTrue(item.body.contains("\"kind\":\"cancel\""))
        val updates = remote.runOnce(3_000L)                         // the cancel goes out ...
        assertTrue(updates.isEmpty() || updates.single().stage != Stage.DELIVERED)
        assertEquals(OutboxState.SUBMITTED, mem.outbox.values.single().state)

        val (m2, e2, r2) = rig(Script(HttpResult(401, "")))
        e2.proceed(checkNotNull(e2.receive(internetPeer, sos(1), 1_000L)))
        val rejected = r2.runOnce(2_000L).single()
        assertEquals(Stage.FAILED, rejected.stage)
        assertEquals("Emergency server rejected the credentials", rejected.note)
        assertEquals(OutboxState.REJECTED, m2.outbox.values.single().state)
        assertTrue(r2.runOnce(1_000_000L).isEmpty())                 // not retried blindly
    }

    @Test fun volatileIdentityUsesASeparateServerKeyAndRecoveryReenqueues() {
        val (mem, engine, _) = rig(Script())
        engine.proceed(checkNotNull(engine.receive(internetPeer, sos(1, flags = 1 or RemoteDelivery.FLAG_IDENTITY_VOLATILE), 1_000L)))
        val item = mem.outbox.values.single()
        assertEquals("$incident-v1000", item.incidentKey)
        mem.outbox.clear()                                           // process killed between record and outbox writes
        engine.recover()
        assertEquals(1, mem.outbox.size)                             // repaired from the durable record
    }

    @Test fun backendConfigRequiresTlsExceptDebugLoopback() {
        val secret = ByteArray(32)
        assertTrue(BackendConfig("https://sos.example.org", "dev-1", secret).valid(false))
        assertFalse(BackendConfig("http://sos.example.org", "dev-1", secret).valid(true))
        assertFalse(BackendConfig("http://10.0.2.2:8443", "dev-1", secret).valid(false))
        assertTrue(BackendConfig("http://10.0.2.2:8443", "dev-1", secret).valid(true))
        assertFalse(BackendConfig("https://sos.example.org", "dev 1", secret).valid(false))
        assertFalse(BackendConfig("https://sos.example.org", "dev-1", ByteArray(8)).valid(false))
    }

    @Test fun monitoringStateIsNeverStale() {
        assertEquals(MonitorStatus.OFF, monitorStatus(false, true, LinkState.READY, true, false, false))
        assertEquals(MonitorStatus.ACTIVE_CONNECTED, monitorStatus(true, true, LinkState.READY, true, false, false))
        assertEquals(MonitorStatus.ACTIVE_DISCONNECTED, monitorStatus(true, true, LinkState.BACKOFF, true, false, false))
        assertEquals(MonitorStatus.PERMISSION_REQUIRED, monitorStatus(true, true, LinkState.READY, false, false, false))
        assertEquals(MonitorStatus.BLUETOOTH_OFF, monitorStatus(true, true, LinkState.DISABLED, true, false, false))
        // requested but the service is gone (process death / force stop / refused restart)
        assertEquals(MonitorStatus.USER_ACTION_REQUIRED, monitorStatus(true, false, LinkState.INACTIVE, true, true, true))
        assertEquals(MonitorStatus.RECOVERY_PENDING, monitorStatus(true, false, LinkState.INACTIVE, true, false, true))
        assertEquals(MonitorStatus.USER_ACTION_REQUIRED, monitorStatus(true, false, LinkState.INACTIVE, true, false, false))
    }

    @Test fun miniJsonEscapesAndReadsServerFields() {
        assertEquals("""{"a":"x\"y\\z","n":1,"b":true,"o":{"d":1.5},"z":null}""",
            MiniJson.obj(linkedMapOf("a" to "x\"y\\z", "n" to 1, "b" to true, "o" to mapOf("d" to 1.5), "z" to null)))
        assertEquals("provider_accepted", MiniJson.string("""{"result":"x","overall" : "provider_accepted"}""", "overall"))
        assertNull(MiniJson.string("""{"overall":5}""", "overall"))
    }

    /** Cross-system: Kotlin client -> real Python backend (loopback HTTP) -> fake provider. No real messages. */
    @Test fun endToEndAgainstTheRealBackendWithItsFakeProvider() {
        val backend = File("../../../Backend").canonicalFile
        val python = listOf("/usr/bin/python3", "/opt/homebrew/bin/python3").firstOrNull { File(it).canExecute() }
        assumeTrue(python != null && File(backend, "paraguardian_sos/server.py").exists())
        val db = File.createTempFile("pg-backend", ".db").apply { delete() }
        val script = """
import sys, threading, json
from paraguardian_sos.config import Config
from paraguardian_sos.store import Store
from paraguardian_sos.providers import FakeProvider
from paraguardian_sos.service import EmergencyService
from paraguardian_sos.server import build_server, Worker
cfg = Config(db_path=sys.argv[1], port=0)
svc = EmergencyService(cfg, Store(cfg.db_path), FakeProvider())
secret = svc.provision_device("pg-dev-1")
svc.set_contacts("pg-dev-1", [{"name": "Test", "e164": "+41790000001", "consent": True}])
httpd = build_server(svc, cfg)
w = Worker(svc, 0.05); w.start()
print(json.dumps({"port": httpd.server_address[1], "secret": secret.hex()}), flush=True)
threading.Thread(target=httpd.serve_forever, daemon=True).start()
sys.stdin.read()
"""
        val process = ProcessBuilder(python, "-c", script, db.path).directory(backend).redirectErrorStream(false).start()
        try {
            val line = process.inputStream.bufferedReader().readLine() ?: error("backend did not start")
            val port = MiniJson.string(line.replace(Regex("\"port\": (\\d+)"), "\"port\": \"$1\""), "port")!!.toInt()
            val secretHex = MiniJson.string(line, "secret")!!
            val live = BackendConfig("http://127.0.0.1:$port", "pg-dev-1",
                secretHex.chunked(2).map { it.toInt(16).toByte() }.toByteArray())
            assertTrue(live.valid(true))
            val mem = Memory()
            val remote = RemoteDelivery({ live }, UrlConnectionTransport(5_000, 5_000), mem)
            val engine = EmergencyEngine(mem, MockDelivery(), object : RemoteQueue {
                override fun handles(record: Record) = true
                override fun enqueue(record: Record) { remote.enqueue(record, true, 0L) }
            })
            engine.proceed(checkNotNull(engine.receive(internetPeer, sos(1), 0L)))
            engine.receive(internetPeer, sos(1), 1L)                       // duplicate: no second submission
            var now = 0L
            val seen = mutableListOf<Stage>()
            repeat(40) {
                remote.runOnce(now).forEach { u -> engine.applyRemote(u)?.let { seen += it.stage } }
                now += RemoteDelivery.POLL_MS
                Thread.sleep(50)
            }
            assertTrue("stages seen: $seen", Stage.SERVER_ACCEPTED in seen || Stage.PROVIDER_ACCEPTED in seen)
            assertEquals(Stage.PROVIDER_ACCEPTED, mem.records.values.single().stage)   // fake provider: no delivery report
        } finally {
            process.destroy()
            process.waitFor(5, TimeUnit.SECONDS)
            db.delete()
        }
    }
}
