package org.paraguardian.companion

import android.annotation.SuppressLint
import android.content.ContentValues
import android.content.SharedPreferences
import androidx.core.database.sqlite.transaction
import android.content.Context
import android.database.Cursor
import android.database.sqlite.SQLiteDatabase
import android.database.sqlite.SQLiteOpenHelper
import org.json.JSONArray
import org.json.JSONObject

class IncidentDatabase(context: Context, name: String = "incidents.db") :
    SQLiteOpenHelper(context, name, null, 2), IncidentStore, OutboxStore {
    override fun onCreate(db: SQLiteDatabase) {
        db.execSQL(
            """CREATE TABLE incidents (
                device TEXT NOT NULL, incident INTEGER NOT NULL, revision INTEGER NOT NULL,
                trigger INTEGER NOT NULL, received INTEGER NOT NULL, event INTEGER NOT NULL,
                route TEXT NOT NULL, position TEXT, raw BLOB, stage TEXT NOT NULL,
                attempted INTEGER NOT NULL, cancelled INTEGER NOT NULL, hidden INTEGER NOT NULL DEFAULT 0,
                composer_targets TEXT,
                PRIMARY KEY (device, incident))"""
        )
        db.execSQL(
            """CREATE TABLE incident_events (
                ordinal INTEGER PRIMARY KEY, device TEXT NOT NULL, incident INTEGER NOT NULL,
                at INTEGER NOT NULL, revision INTEGER NOT NULL, stage TEXT NOT NULL, cancelled INTEGER NOT NULL)"""
        )
        db.execSQL("CREATE INDEX incident_events_key ON incident_events(device, incident)")
        createOutbox(db)
    }

    /** Version 2 (Milestone 8): durable outbox for internet delivery. */
    private fun createOutbox(db: SQLiteDatabase) {
        db.execSQL(
            """CREATE TABLE outbox (
                device TEXT NOT NULL, incident INTEGER NOT NULL, incident_key TEXT NOT NULL, revision INTEGER NOT NULL,
                kind TEXT NOT NULL, body TEXT NOT NULL, test INTEGER NOT NULL, state TEXT NOT NULL,
                attempts INTEGER NOT NULL, next_attempt INTEGER NOT NULL, overall TEXT, last_error TEXT,
                created INTEGER NOT NULL, PRIMARY KEY (device, incident))"""
        )
    }

    override fun onConfigure(db: SQLiteDatabase) {
        db.rawQuery("PRAGMA secure_delete=ON", null).use { it.moveToFirst() }
    }

    override fun onUpgrade(db: SQLiteDatabase, oldVersion: Int, newVersion: Int) {
        if (oldVersion == 1 && newVersion == 2) createOutbox(db)   // existing incidents are kept untouched
        else error("Missing incident database migration $oldVersion to $newVersion")
    }

    override fun outbox(key: Key): OutboxItem? = readableDatabase.query(
        "outbox", null, "device=? AND incident=?", arrayOf(key.device, key.incident.toString()), null, null, null
    ).use { if (it.moveToFirst()) it.outboxItem() else null }

    override fun putOutbox(item: OutboxItem) {
        val values = ContentValues().apply {
            put("device", item.key.device); put("incident", item.key.incident); put("incident_key", item.incidentKey)
            put("revision", item.revision); put("kind", item.kind); put("body", item.body); put("test", item.test)
            put("state", item.state.name); put("attempts", item.attempts); put("next_attempt", item.nextAttemptAt)
            put("overall", item.overall); put("last_error", item.lastError); put("created", item.createdAt)
        }
        check(writableDatabase.insertWithOnConflict("outbox", null, values, SQLiteDatabase.CONFLICT_REPLACE) != -1L)
    }

    override fun outboxItems(): List<OutboxItem> = readableDatabase.query(
        "outbox", null, null, null, null, null, "created"
    ).use { c -> buildList { while (c.moveToNext()) add(c.outboxItem()) } }

    private fun Cursor.outboxItem() = OutboxItem(
        Key(getString(0), getLong(1)), getString(2), getInt(3), getString(4), getString(5), getInt(6) != 0,
        OutboxState.valueOf(getString(7)), getInt(8), getLong(9), if (isNull(10)) null else getString(10),
        if (isNull(11)) null else getString(11), getLong(12)
    )

    override fun get(key: Key): Record? = readableDatabase.query(
        "incidents", null, "device=? AND incident=?", arrayOf(key.device, key.incident.toString()),
        null, null, null
    ).use { if (it.moveToFirst()) it.record() else null }

    override fun put(record: Record) {
        val selection = arrayOf(record.key.device, record.key.incident.toString())
        writableDatabase.transaction {
            val db = this
            val prior = db.query("incidents", arrayOf("hidden", "revision", "stage", "cancelled"),
                "device=? AND incident=?", selection, null, null, null).use {
                if (it.moveToFirst()) Prior(it.getInt(0) != 0, it.getInt(1), it.getString(2), it.getInt(3) != 0)
                else null
            }
            val hidden = prior?.hidden == true
            val values = ContentValues().apply {
                put("device", record.key.device)
                put("incident", record.key.incident)
                put("revision", record.revision)
                put("trigger", record.trigger)
                put("received", record.received)
                put("event", record.eventTime)
                put("route", record.route.name)
                if (!hidden) {
                    put("position", record.position?.toJson()?.toString())
                    put("raw", record.raw)
                    put("composer_targets", record.composerTargets.joinToString(","))
                }
                put("stage", record.stage.name)
                put("attempted", record.attempted)
                put("cancelled", record.cancelledNotice)
                put("hidden", hidden)
            }
            db.insertWithOnConflict("incidents", null, values, SQLiteDatabase.CONFLICT_REPLACE)
                .also { check(it != -1L) }
            if (!hidden && (prior == null || prior.revision != record.revision || prior.stage != record.stage.name ||
                    prior.cancelled != record.cancelledNotice)
            ) {
                db.insertOrThrow("incident_events", null, ContentValues().apply {
                    put("device", record.key.device)
                    put("incident", record.key.incident)
                    put("at", System.currentTimeMillis())
                    put("revision", record.revision)
                    put("stage", record.stage.name)
                    put("cancelled", record.cancelledNotice)
                })
            }
        }
    }

    private data class Prior(val hidden: Boolean, val revision: Int, val stage: String, val cancelled: Boolean)

    data class Timeline(val at: Long, val revision: Int, val stage: Stage, val cancelled: Boolean)

    fun timeline(): Map<Key, List<Timeline>> = readableDatabase.query(
        "incident_events", arrayOf("device", "incident", "at", "revision", "stage", "cancelled"),
        null, null, null, null, "ordinal DESC", "400"
    ).use { cursor ->
        val result = mutableMapOf<Key, MutableList<Timeline>>()
        while (cursor.moveToNext()) {
            result.getOrPut(Key(cursor.getString(0), cursor.getLong(1))) { mutableListOf() }.add(
                Timeline(cursor.getLong(2), cursor.getInt(3), Stage.valueOf(cursor.getString(4)), cursor.getInt(5) != 0)
            )
        }
        result
    }

    override fun recoverable(): List<Record> =
        list("stage IN ('STORED','ATTEMPTING','SUBMISSION_PENDING','SERVER_ACCEPTED')")
    fun visible(): List<Record> = list("hidden=0", "200")

    private fun list(selection: String?, limit: String? = null): List<Record> = readableDatabase.query(
        "incidents", null, selection, null, null, null, "received DESC", limit
    ).use { cursor -> buildList { while (cursor.moveToNext()) add(cursor.record()) } }

    fun redact(now: Long, all: Boolean) {
        val where = if (all) "hidden=0" else "received < ? AND hidden=0"
        val args = if (all) emptyArray() else arrayOf((now - 30L * 24 * 60 * 60 * 1000).toString())
        val values = ContentValues().apply {
            putNull("position")
            putNull("raw")
            putNull("composer_targets")
            put("hidden", true)
        }
        writableDatabase.transaction {
            update("incidents", values, where, args)
            execSQL("DELETE FROM incident_events WHERE EXISTS (SELECT 1 FROM incidents i " +
                "WHERE i.hidden=1 AND i.device=incident_events.device AND i.incident=incident_events.incident)")
        }
    }

    private fun Cursor.record() = Record(
        Key(getString(0), getLong(1)), getInt(2), getInt(3), getLong(4), getLong(5),
        Route.valueOf(getString(6)), if (isNull(7)) null else JSONObject(getString(7)).position(),
        getBlob(8), Stage.valueOf(getString(9)), getInt(10) != 0, getInt(11) != 0,
        getString(13)?.split(',')?.filterNot(String::isBlank)?.toSet().orEmpty()
    )
}

private fun Position.toJson() = JSONObject().apply {
    put("lat", lat)
    put("lon", lon)
    put("age", ageMs)
    put("accuracy", accuracyM)
    put("source", source)
    put("stale", stale)
    put("acquired", acquiredForIncident)
    put("captured", capturedEpochMs)
}

private fun JSONObject.position() = Position(
    getDouble("lat"), getDouble("lon"), getLong("age"),
    optDouble("accuracy").takeIf { it.isFinite() }, getString("source"), getBoolean("stale"),
    optBoolean("acquired"), optLong("captured").takeIf { has("captured") }
)

/**
 * Durable preference write: the KTX edit {} extension discards commit()'s result, but a failed
 * write of monitoring / delivery settings must be detected, so the Boolean is checked here.
 */
@SuppressLint("UseKtx", "ApplySharedPref")
private fun SharedPreferences.commitChange(block: SharedPreferences.Editor.() -> Unit) =
    check(edit().apply(block).commit()) { "preference write failed" }

class Settings(context: Context) {
    private val prefs = context.getSharedPreferences("setup_v1", Context.MODE_PRIVATE)
    var address: String?
        get() = prefs.getString("address", null)
        set(value) { prefs.commitChange { putString("address", value) } }
    var lastPhysicalConnection: Long
        get() = prefs.getLong("lastPhysicalConnection", 0L)
        set(value) { prefs.commitChange { putLong("lastPhysicalConnection", value) } }
    var simulation: Boolean
        get() = prefs.getBoolean("simulation", true)
        set(value) { prefs.commitChange { putBoolean("simulation", value) } }
    var template: String
        get() = prefs.getString("template", "").orEmpty()
        set(value) { prefs.commitChange { putString("template", value.take(160)) } }
    var safetyNotifications: Boolean
        get() = prefs.getBoolean("safetyNotifications", true)
        set(value) { prefs.commitChange { putBoolean("safetyNotifications", value) } }

    /** What the user ASKED for (persisted); whether monitoring actually runs is ViewState.monitoring. */
    var monitoringRequested: Boolean
        get() = prefs.getBoolean("monitoringRequested", false)
        set(value) { prefs.commitChange { putBoolean("monitoringRequested", value) } }
    /** Set when Android refused a background restart: the user must reopen the app. */
    var recoveryPending: Boolean
        get() = prefs.getBoolean("recoveryPending", false)
        set(value) { prefs.commitChange { putBoolean("recoveryPending", value) } }
    var internetDelivery: Boolean
        get() = prefs.getBoolean("internetDelivery", false)
        set(value) { prefs.commitChange { putBoolean("internetDelivery", value) } }
    var simulationUsesBackend: Boolean
        get() = prefs.getBoolean("simulationUsesBackend", false)
        set(value) { prefs.commitChange { putBoolean("simulationUsesBackend", value) } }

    fun contacts(): List<Contact> {
        val list = JSONArray(prefs.getString("contacts", "[]"))
        return (0 until list.length()).map { index ->
            val c = list.getJSONObject(index)
            Contact(c.getString("name"), c.getString("number"), c.getBoolean("enabled"), c.getBoolean("consent"))
        }
    }

    fun saveContacts(contacts: List<Contact>) {
        require(contacts.size <= 5 && contacts.all { it.valid } &&
            contacts.map { it.number }.distinct().size == contacts.size)
        val list = JSONArray()
        contacts.forEach { c ->
            list.put(JSONObject().put("name", c.name).put("number", c.number)
                .put("enabled", c.enabled).put("consent", c.consent))
        }
        prefs.commitChange { putString("contacts", list.toString()) }
    }
}
