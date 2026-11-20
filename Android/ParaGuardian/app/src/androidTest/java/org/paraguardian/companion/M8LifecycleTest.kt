package org.paraguardian.companion

import android.content.Context
import android.database.sqlite.SQLiteDatabase
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith

/** Milestone 8 on a real Android runtime: Keystore credentials, database v1 -> v2 migration, outbox durability,
 *  and the monitoring starter's refusal rules. */
@RunWith(AndroidJUnit4::class)
class M8LifecycleTest {
    private val context: Context get() = InstrumentationRegistry.getInstrumentation().targetContext

    @Test fun keystoreCredentialsRoundTripAndPlaintextIsNotStored() {
        val creds = BackendCredentials(context)
        creds.clear()
        val secret = "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff"
        assertFalse(creds.save("http://sos.example.org", "pg-dev-1", secret))   // no plain HTTP off-loopback
        assertTrue(creds.save("https://sos.example.org", "pg-dev-1", secret))
        val loaded = checkNotNull(creds.load())
        assertEquals("pg-dev-1", loaded.deviceId)
        assertEquals(secret, loaded.secret.joinToString("") { "%02x".format(it.toInt() and 255) })
        val stored = context.getSharedPreferences("backend_v1", Context.MODE_PRIVATE).all.values.joinToString()
        assertFalse(stored.contains(secret))                                  // only ciphertext at rest
        assertTrue(creds.clear())
        assertNull(creds.load())
    }

    @Test fun databaseMigrationKeepsIncidentsAndOutboxSurvivesReopen() {
        val name = "m8-migration-test.db"
        context.deleteDatabase(name)
        val path = context.getDatabasePath(name).apply { parentFile?.mkdirs() }
        SQLiteDatabase.openOrCreateDatabase(path, null).use { db ->   // a Milestone 7 (version 1) database
            db.execSQL("""CREATE TABLE incidents (device TEXT NOT NULL, incident INTEGER NOT NULL, revision INTEGER NOT NULL,
                trigger INTEGER NOT NULL, received INTEGER NOT NULL, event INTEGER NOT NULL, route TEXT NOT NULL,
                position TEXT, raw BLOB, stage TEXT NOT NULL, attempted INTEGER NOT NULL, cancelled INTEGER NOT NULL,
                hidden INTEGER NOT NULL DEFAULT 0, composer_targets TEXT, PRIMARY KEY (device, incident))""")
            db.execSQL("""CREATE TABLE incident_events (ordinal INTEGER PRIMARY KEY, device TEXT NOT NULL,
                incident INTEGER NOT NULL, at INTEGER NOT NULL, revision INTEGER NOT NULL, stage TEXT NOT NULL,
                cancelled INTEGER NOT NULL)""")
            db.execSQL("INSERT INTO incidents VALUES ('sim', 5, 1, 1, 1, 1, 'SIMULATION', NULL, NULL, 'DELIVERED', 1, 0, 0, '')")
            db.version = 1
        }
        try {
            val key = Key("dev", 9)
            IncidentDatabase(context, name).use { db ->
                assertEquals(Stage.DELIVERED, db.get(Key("sim", 5))!!.stage)   // M7 incident preserved
                db.putOutbox(OutboxItem(key, "9", 1, "alert", "{}", false, OutboxState.PENDING, 0, 0, null, null, 0))
            }
            IncidentDatabase(context, name).use { db ->                        // process restart
                assertEquals(OutboxState.PENDING, db.outbox(key)!!.state)
                assertEquals(1, db.outboxItems().size)
            }
        } finally { context.deleteDatabase(name) }
    }

    @Test fun monitoringIsNotStartedUnlessRequested() {
        val settings = Settings(context)
        val before = settings.monitoringRequested
        try {
            settings.monitoringRequested = false
            assertFalse(MonitoringStarter.tryStart(context, "test"))
        } finally { settings.monitoringRequested = before }
    }
}
