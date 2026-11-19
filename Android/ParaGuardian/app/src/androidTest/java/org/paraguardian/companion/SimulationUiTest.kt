package org.paraguardian.companion

import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.hasText
import androidx.compose.ui.test.junit4.createAndroidComposeRule
import androidx.compose.ui.test.onAllNodesWithText
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performScrollTo
import org.junit.Rule
import org.junit.Test

class SimulationUiTest {
    @get:Rule val compose = createAndroidComposeRule<MainActivity>()

    @Test fun automaticSosReachesVisibleMockDelivery() {
        val guardian = (compose.activity.application as GuardianApplication).guardian
        compose.waitUntil(10_000) { guardian.state.value.storageReady }
        compose.runOnIdle {
            guardian.simulationConnect(false)
            guardian.simulationAction("New simulated device")
        }
        compose.onNodeWithText("Connect simulator").performScrollTo().performClick()
        compose.onAllNodesWithText("Emergency")[0].performClick()
        compose.onNodeWithText("Flight").performScrollTo().performClick()
        compose.onNodeWithText("Suspect crash").performScrollTo().performClick()
        compose.waitUntil(10_000) { guardian.state.value.status?.state == 1 }
        compose.onNodeWithText("Advance 31 seconds").performScrollTo().performClick()
        compose.waitUntil(10_000) { guardian.state.value.status?.state == 5 }
        compose.onNode(hasText("Phone record #1 · Delivery reported")).performScrollTo().assertIsDisplayed()
        compose.onNodeWithText("Simulation · No external messages").assertIsDisplayed()
    }
}
