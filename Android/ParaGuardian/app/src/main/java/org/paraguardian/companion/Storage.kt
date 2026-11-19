package org.paraguardian.companion

import android.content.ContentValues
import android.content.Context
import android.database.Cursor
import android.database.sqlite.SQLiteDatabase
import android.database.sqlite.SQLiteOpenHelper
import org.json.JSONArray
import org.json.JSONObject

class IncidentDatabase(context: Context, name: String = "incidents.db") :
    SQLiteOpenHelper(context, name, null, 1), IncidentStore {
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
    }

    override fun onConfigure(db: SQLiteDatabase) {
        db.rawQuery("PRAGMA secure_delete=ON", null).use { it.moveToFirst() }
    }

    override fun onUpgrade(db: SQLiteDatabase, oldVersion: Int, newVersion: Int) {
        error("Missing incident database migration $oldVersion to $newVersion")
    }

    override fun get(key: Key): Record? = readableDatabase.query(
        "incidents", null, "device=? AND incident=?", arrayOf(key.device, key.incident.toString()),
        null, null, null
    ).use { if (it.moveToFirst()) it.record() else null }

    override fun put(record: Record) {
        val selection = arrayOf(record.key.device, record.key.incident.toString())
        val db = writableDatabase
        db.beginTransaction()
        try {
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
            db.setTransactionSuccessful()
        } finally {
            db.endTransaction()
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

    override fun recoverable(): List<Record> = list("stage IN ('STORED','ATTEMPTING')")
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
        val db = writableDatabase
        db.beginTransaction()
        try {
            db.update("incidents", values, where, args)
            db.execSQL("DELETE FROM incident_events WHERE EXISTS (SELECT 1 FROM incidents i " +
                "WHERE i.hidden=1 AND i.device=incident_events.device AND i.incident=incident_events.incident)")
            db.setTransactionSuccessful()
        } finally { db.endTransaction() }
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

class Settings(context: Context) {
    private val prefs = context.getSharedPreferences("setup_v1", Context.MODE_PRIVATE)
    var address: String?
        get() = prefs.getString("address", null)
        set(value) { check(prefs.edit().putString("address", value).commit()) }
    var lastPhysicalConnection: Long
        get() = prefs.getLong("lastPhysicalConnection", 0L)
        set(value) { check(prefs.edit().putLong("lastPhysicalConnection", value).commit()) }
    var simulation: Boolean
        get() = prefs.getBoolean("simulation", true)
        set(value) { check(prefs.edit().putBoolean("simulation", value).commit()) }
    var template: String
        get() = prefs.getString("template", "").orEmpty()
        set(value) { check(prefs.edit().putString("template", value.take(160)).commit()) }
    var safetyNotifications: Boolean
        get() = prefs.getBoolean("safetyNotifications", true)
        set(value) { check(prefs.edit().putBoolean("safetyNotifications", value).commit()) }

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
        check(prefs.edit().putString("contacts", list.toString()).commit())
    }
}
