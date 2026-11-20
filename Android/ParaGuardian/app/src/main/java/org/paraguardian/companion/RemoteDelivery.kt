package org.paraguardian.companion

import java.net.HttpURLConnection
import java.net.SocketTimeoutException
import java.net.URL
import java.security.MessageDigest
import javax.crypto.Mac
import javax.crypto.spec.SecretKeySpec

/**
 * Unattended internet delivery: Android -> HTTPS -> ParaGuardian emergency API -> provider -> contacts.
 *
 * Durable client-side submission: every SOS revision becomes an outbox row BEFORE any network call.
 * The worker (process start, foreground monitoring, JobScheduler job after reboot / when the network
 * returns) submits due rows, retries transient failures with bounded backoff, and polls the server for
 * the delivery outcome. Retries are safe: the server de-duplicates on (device, incidentKey, revision).
 *
 * Honest stages: the server accepting a request is not a send; a provider accepting it is not delivery;
 * only a provider delivery report becomes DELIVERED, and every uncertain case stays UNKNOWN.
 */
data class BackendConfig(val baseUrl: String, val deviceId: String, val secret: ByteArray) {
    fun valid(debugLoopbackAllowed: Boolean): Boolean {
        val url = runCatching { URL(baseUrl) }.getOrNull() ?: return false
        val loopback = url.host in setOf("10.0.2.2", "127.0.0.1", "localhost")
        val schemeOk = url.protocol == "https" || (debugLoopbackAllowed && loopback && url.protocol == "http")
        return schemeOk && deviceId.matches(Regex("[A-Za-z0-9_.:-]{3,64}")) && secret.size >= 16
    }
}

/** Minimal JSON for the delivery API (keeps RemoteDelivery runnable in plain JVM unit tests). */
object MiniJson {
    fun quote(s: String): String = buildString {
        append('"')
        for (c in s) when {
            c == '"' -> append("\\\"")
            c == '\\' -> append("\\\\")
            c < ' ' -> append("\\u%04x".format(c.code))
            else -> append(c)
        }
        append('"')
    }

    /** Values: String, Number, Boolean, null, or a nested Map. Keys keep insertion order. */
    fun obj(map: Map<String, Any?>): String = map.entries.joinToString(",", "{", "}") { (k, v) ->
        quote(k) + ":" + when (v) {
            null -> "null"
            is String -> quote(v)
            is Boolean, is Int, is Long -> v.toString()
            is Double -> if (v.isFinite()) v.toString() else "null"
            is Map<*, *> -> @Suppress("UNCHECKED_CAST") obj(v as Map<String, Any?>)
            else -> quote(v.toString())
        }
    }

    /** Top-level string field of a server answer (the server's JSON has no nested "overall"). */
    fun string(json: String, field: String): String? =
        Regex("\"" + Regex.escape(field) + "\"\\s*:\\s*\"([^\"\\\\]*)\"").find(json)?.groupValues?.get(1)
}

object RequestSigner {
    private fun hex(bytes: ByteArray) = bytes.joinToString("") { "%02x".format(it.toInt() and 255) }

    fun bodyHash(body: ByteArray): String = hex(MessageDigest.getInstance("SHA-256").digest(body))

    /** Same canonical string as Backend/paraguardian_sos/auth.py. */
    fun sign(secret: ByteArray, method: String, path: String, timestamp: Long, nonce: String, body: ByteArray): String {
        val canonical = listOf(method.uppercase(), path, timestamp.toString(), nonce, bodyHash(body)).joinToString("\n")
        val mac = Mac.getInstance("HmacSHA256")
        mac.init(SecretKeySpec(secret, "HmacSHA256"))
        return hex(mac.doFinal(canonical.toByteArray(Charsets.UTF_8)))
    }
}

/** code: HTTP status, or NETWORK (nothing reached the server) / TIMEOUT (it may have). */
data class HttpResult(val code: Int, val body: String) {
    companion object {
        const val NETWORK = -1
        const val TIMEOUT = -2
    }
}

fun interface HttpTransport {
    fun exchange(method: String, url: String, headers: Map<String, String>, body: ByteArray?): HttpResult
}

/** HttpsURLConnection with platform TLS verification (system trust store, hostname check). */
class UrlConnectionTransport(private val connectTimeoutMs: Int = 10_000, private val readTimeoutMs: Int = 15_000) :
    HttpTransport {
    override fun exchange(method: String, url: String, headers: Map<String, String>, body: ByteArray?): HttpResult {
        val connection = try { URL(url).openConnection() as HttpURLConnection } catch (_: Exception) {
            return HttpResult(HttpResult.NETWORK, "")
        }
        return try {
            connection.requestMethod = method
            connection.connectTimeout = connectTimeoutMs
            connection.readTimeout = readTimeoutMs
            connection.instanceFollowRedirects = false
            connection.useCaches = false
            headers.forEach { (k, v) -> connection.setRequestProperty(k, v) }
            if (body != null) {
                connection.doOutput = true
                connection.setFixedLengthStreamingMode(body.size)
                connection.outputStream.use { it.write(body) }
            }
            val code = connection.responseCode
            val stream = if (code >= 400) connection.errorStream else connection.inputStream
            val text = stream?.use { it.readBytes().take(65_536).toByteArray().toString(Charsets.UTF_8) }.orEmpty()
            HttpResult(code, text)
        } catch (_: SocketTimeoutException) {
            HttpResult(HttpResult.TIMEOUT, "")
        } catch (_: java.io.IOException) {
            // Connection refused / reset / TLS failure. If the body was already written the server may
            // have it; a later status poll (same key) reconciles it, and resubmission is idempotent.
            HttpResult(HttpResult.NETWORK, "")
        } finally {
            connection.disconnect()
        }
    }
}

enum class OutboxState { PENDING, SUBMITTED, DONE, REJECTED }

data class OutboxItem(
    val key: Key, val incidentKey: String, val revision: Int, val kind: String, val body: String,
    val test: Boolean, val state: OutboxState, val attempts: Int, val nextAttemptAt: Long,
    val overall: String?, val lastError: String?, val createdAt: Long
)

interface OutboxStore {
    fun outbox(key: Key): OutboxItem?
    fun putOutbox(item: OutboxItem)
    fun outboxItems(): List<OutboxItem>
}

/** One change the caller applies to the incident record (and reports to the MCU). */
data class RemoteUpdate(val key: Key, val stage: Stage, val note: String)

class RemoteDelivery(
    private val config: () -> BackendConfig?,
    private val transport: HttpTransport,
    private val store: OutboxStore,
    private val nonce: () -> String = { java.util.UUID.randomUUID().toString().replace("-", "") },
) {
    companion object {
        const val POLL_MS = 15_000L
        const val POLL_FOR_MS = 24L * 60 * 60 * 1000
        const val MAX_BACKOFF_MS = 300_000L
        const val FLAG_IDENTITY_VOLATILE = 1 shl 8
        const val FLAG_RESTORED = 1 shl 9

        fun stageFor(overall: String?): Stage? = when (overall) {
            "pending" -> Stage.SERVER_ACCEPTED
            "provider_accepted" -> Stage.PROVIDER_ACCEPTED
            "delivered" -> Stage.DELIVERED
            "failed", "no_recipients" -> Stage.FAILED
            "unknown" -> Stage.UNKNOWN
            "cancelled" -> Stage.CANCELLED
            else -> null
        }

        fun backoff(attempts: Int): Long = (5_000L shl (attempts - 1).coerceIn(0, 6)).coerceAtMost(MAX_BACKOFF_MS)
    }

    var lastNetworkProblem: String? = null
        private set

    /** Durable before any network call. Idempotent: an older / equal revision changes nothing. */
    fun enqueue(record: Record, test: Boolean, now: Long): OutboxItem? {
        val raw = record.raw ?: return null
        val sos = Protocol.sos(Protocol.Frame(Protocol.SOS, 0, record.key.incident, raw)) ?: return null
        val previous = store.outbox(record.key)
        if (previous != null && previous.revision >= record.revision) return previous
        val volatile = sos.flags and FLAG_IDENTITY_VOLATILE != 0
        val incidentKey = previous?.incidentKey
            ?: if (volatile) "${record.key.incident}-v${record.received}" else record.key.incident.toString()
        val kind = if (record.cancelledNotice) "cancel" else "alert"
        val fields = linkedMapOf<String, Any?>(
            "incidentKey" to incidentKey, "incidentId" to record.key.incident, "revision" to record.revision,
            "kind" to kind, "trigger" to record.trigger.coerceIn(0, 4), "test" to test,
            "identityVolatile" to volatile, "restored" to (sos.flags and FLAG_RESTORED != 0)
        )
        record.position?.let { p ->
            fields["position"] = linkedMapOf<String, Any?>("lat" to p.lat, "lon" to p.lon,
                "ageMs" to p.ageMs.coerceAtLeast(0), "source" to if (p.source == "Device GNSS") "gnss" else "phone")
                .also { m -> p.accuracyM?.let { m["accuracyM"] = it } }
        }
        val body = MiniJson.obj(fields)
        val item = OutboxItem(record.key, incidentKey, record.revision, kind, body, test, OutboxState.PENDING,
            0, now, previous?.overall, null, previous?.createdAt ?: now)
        store.putOutbox(item)
        return item
    }

    fun nextWakeAt(): Long? = store.outboxItems()
        .filter { it.state == OutboxState.PENDING || it.state == OutboxState.SUBMITTED }
        .minOfOrNull { it.nextAttemptAt }

    /** Processes every due item once. Bounded by the number of open items. */
    fun runOnce(now: Long): List<RemoteUpdate> {
        val cfg = config() ?: return emptyList()
        val updates = mutableListOf<RemoteUpdate>()
        for (item in store.outboxItems()) {
            if (item.nextAttemptAt > now) continue
            when (item.state) {
                OutboxState.PENDING -> submit(cfg, item, now)?.let(updates::add)
                OutboxState.SUBMITTED -> poll(cfg, item, now)?.let(updates::add)
                else -> Unit
            }
        }
        return updates
    }

    private fun call(cfg: BackendConfig, method: String, path: String, body: ByteArray?): HttpResult {
        val ts = System.currentTimeMillis() / 1000
        val n = nonce()
        val headers = mapOf(
            "X-PG-Device" to cfg.deviceId, "X-PG-Timestamp" to ts.toString(), "X-PG-Nonce" to n,
            "X-PG-Signature" to RequestSigner.sign(cfg.secret, method, path, ts, n, body ?: ByteArray(0)),
            "Content-Type" to "application/json", "Accept" to "application/json"
        )
        return transport.exchange(method, cfg.baseUrl.trimEnd('/') + path, headers, body)
    }

    private fun submit(cfg: BackendConfig, item: OutboxItem, now: Long): RemoteUpdate? {
        val result = call(cfg, "POST", "/v1/incidents", item.body.toByteArray(Charsets.UTF_8))
        return when {
            result.code == 200 || result.code == 202 -> {
                lastNetworkProblem = null
                val overall = MiniJson.string(result.body, "overall")
                store.putOutbox(item.copy(state = OutboxState.SUBMITTED, overall = overall, attempts = item.attempts + 1,
                    nextAttemptAt = now + POLL_MS, lastError = null))
                RemoteUpdate(item.key, stageFor(overall) ?: Stage.SERVER_ACCEPTED, "Emergency server accepted the request")
            }
            result.code in setOf(400, 401, 403, 404, 409, 413) -> {
                // Not retried automatically: the request (or the credentials) must change first.
                store.putOutbox(item.copy(state = OutboxState.REJECTED, attempts = item.attempts + 1,
                    lastError = "HTTP ${result.code}"))
                RemoteUpdate(item.key, Stage.FAILED,
                    if (result.code == 401) "Emergency server rejected the credentials" else "Emergency server rejected the request")
            }
            else -> {   // network down, timeout, 429, 5xx: retry with bounded backoff (idempotent server)
                lastNetworkProblem = when (result.code) {
                    HttpResult.NETWORK -> "Internet unavailable"
                    HttpResult.TIMEOUT -> "Emergency server timed out"
                    else -> "Emergency server busy (HTTP ${result.code})"
                }
                val attempts = item.attempts + 1
                store.putOutbox(item.copy(attempts = attempts, nextAttemptAt = now + backoff(attempts),
                    lastError = lastNetworkProblem))
                RemoteUpdate(item.key, Stage.SUBMISSION_PENDING, lastNetworkProblem!!)
            }
        }
    }

    private fun poll(cfg: BackendConfig, item: OutboxItem, now: Long): RemoteUpdate? {
        val result = call(cfg, "GET", "/v1/incidents/${item.incidentKey}", null)
        if (result.code != 200) {
            store.putOutbox(item.copy(nextAttemptAt = now + POLL_MS * 2))
            return null
        }
        lastNetworkProblem = null
        val overall = MiniJson.string(result.body, "overall")
        val finished = overall in setOf("delivered", "failed", "cancelled", "no_recipients") ||
            now - item.createdAt > POLL_FOR_MS
        store.putOutbox(item.copy(overall = overall, state = if (finished) OutboxState.DONE else OutboxState.SUBMITTED,
            nextAttemptAt = now + POLL_MS))
        if (overall == item.overall) return null
        val stage = stageFor(overall) ?: return null
        return RemoteUpdate(item.key, stage, "Server status: $overall")
    }
}
