package org.paraguardian.companion

import android.Manifest
import android.content.Context
import androidx.test.core.app.ActivityScenario
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
class PersistenceTest {
    private val context: Context get() = InstrumentationRegistry.getInstrumentation().targetContext

    @Test fun processLikeDatabaseRecreationPreservesAttemptsAndRedactsHistory() {
        val name = "m7-recovery-test.db"
        context.deleteDatabase(name)
        val key = Key("simulation-test", 21)
        try {
            IncidentDatabase(context, name).use { db ->
                db.put(Record(key, 1, 1, 1, 1, Route.SIMULATION,
                    Position(51.0, -115.0, 100, 4.0, "Test GNSS", false),
                    byteArrayOf(1, 2), Stage.ATTEMPTING, true))
            }
            val mock = MockDelivery()
            IncidentDatabase(context, name).use { db ->
                EmergencyEngine(db, mock).recover()
                assertEquals(Stage.UNKNOWN, db.get(key)!!.stage)
                assertEquals(0, mock.attempts)
                assertEquals(listOf(Stage.UNKNOWN, Stage.ATTEMPTING), db.timeline()[key]?.map { it.stage })
                db.redact(10_000, true)
            }
            IncidentDatabase(context, name).use { db ->
                assertTrue(db.visible().isEmpty())
                assertTrue(db.timeline().isEmpty())
                assertNull(db.get(key)!!.position)
                assertNull(db.get(key)!!.raw)
                assertEquals(listOf(5, 7, 11), db.get(key)!!.reports())
            }
        } finally { context.deleteDatabase(name) }
    }

    @Test fun activityRecreationLeavesPhysicalMonitoringInactive() {
        ActivityScenario.launch(MainActivity::class.java).use { scenario ->
            scenario.recreate()
            scenario.onActivity { activity ->
                val guardian = (activity.application as GuardianApplication).guardian
                assertFalse(guardian.state.value.monitoring)
                assertEquals(LinkState.INACTIVE, guardian.state.value.link)
            }
        }
    }

    @Test fun freshInstallWithoutNotificationPermissionDoesNotNotify() {
        if (android.os.Build.VERSION.SDK_INT < 33 || context.granted(Manifest.permission.POST_NOTIFICATIONS)) return
        assertFalse(GuardianNotifications(context).alert("Simulation", "Instrumentation never contacts anyone", true))
    }
}
