# Flight State, LiDAR Ground Clearance and Landing Safety: Design (Milestone 5)

**Status:** implemented, integrated into `SensorProcessingTask`, validated in **host simulation only**.
No PCB, LiDAR mounting, sensor or flight data exist. Nothing here is hardware-validated, flight-validated or
safety-qualified.

* **Thresholds:** every threshold is an **initial development parameter**. None is a validated "safe"
  descent rate, clearance or flare height for any wing or pilot.
* **Pilot judgement:** the algorithms do not replace pilot judgement, approved equipment or established
  flying procedures.

Related: `FLIGHT_LANDING_VALIDATION_REPORT.md` (results), `FLIGHT_LANDING_WALKTHROUGH.txt` (teaching
text), `FLIGHT_LANDING_GIT_SUMMARY.md` (files), `VERTICAL_UKF_DESIGN.md` (navigation input),
`MADGWICK_ATTITUDE_DESIGN.md` (attitude input).

---

## 1. Scope

| Delivered | Not delivered (next packages) |
|---|---|
| `FlightStateDetector`: evidence-based phase state machine | Crash detection |
| `GroundClearanceEstimator`: TF02-Pro range + attitude → clearance, closure rate, TTC | Automatic SOS / emergency manager |
| `LandingSafetyAnalyzer`: descent, clearance, closure and degradation warnings | Audio / LED playback, BLE / Android |
| `SafetyEvent`, `SafetyEventEmitter`, `SafetyEventQueue`, `SafetySnapshot` (+ mailbox) | Logging to W25Q128 |
| `SafetyMonitor` orchestration; `SensorProcessingTask` integration; health counters | Flare guidance (deliberately **not** implemented, §8.6) |
| Independent flight/terrain/LiDAR simulator; 55 tests; sweeps | Hardware mounting verification |

---

## 2. Architecture

```
SensorProcessingTask (prio 32), one wake per frame:
  PreprocessingPipeline -> ProcessedSensorFrame
  AttitudeEstimator     -> AttitudeEstimate          (q_ENU_from_body)
  VerticalNavigationEstimator -> NavigationEstimate  (UKF, placeholder model)
  safetyInputFrom(frame, attitude, navigation) -> SafetyInput (140 B)
  SafetyMonitor::update(input):
      1. GroundClearanceEstimator   LiDAR + attitude of THIS frame
      2. FlightStateDetector         navigation + clearance(1) + GNSS + IMU motion
      3. phase-change Notice event
      4. LandingSafetyAnalyzer       phase(2) + clearance(1) + navigation -> warnings + events
  -> SafetySnapshotMailbox (latest state)      -> future audio / LED / UI (repeat while active)
  -> SafetyEventQueue<16> (transitions only)   -> ONE future consumer task (audio or logger)
```

**Order and time semantics:**
* Clearance does not depend on the phase, and the phase does not depend on warnings, so there is no
  circular dependency.
* Each quantity keeps its own timestamp (LiDAR sample, attitude orientation, navigation filter time,
  GNSS solution). Nothing is interpolated. A LiDAR sample is projected only with an attitude within
  40 ms of it.

### Files

| Folder / file | Role |
|---|---|
| `Algorithms/Safety/SafetyInput.hpp` | `SafetyInput` (nav / attitude / IMU / LiDAR / GNSS sub-structs, each with own time and validity) + `safetyInputFrom()` |
| `Algorithms/Safety/SafetyEvent.hpp` | `SafetyEvent` (40 B), types, severities, transitions, sources, `safetyq::*` flags, `ISafetyEventSink` |
| `Algorithms/Safety/SafetyEventEmitter.hpp` | event / episode numbering, up to 4 sinks |
| `Algorithms/Safety/SafetyEventQueue.hpp` | bounded priority-aware FIFO |
| `Algorithms/Safety/SafetySnapshotMailbox.hpp` | latest-value hand-over of the snapshot |
| `Algorithms/Safety/SafetyMonitor.hpp/.cpp` | orchestration, snapshot, phase notices; `SafetyConfig` |
| `Algorithms/Flight/FlightStateTypes.hpp` | `FlightPhase`, `EvidenceLevel`, evidence bits, `FlightStateConfig`, `FlightStateStatus` |
| `Algorithms/Flight/FlightStateDetector.hpp/.cpp` | the phase state machine |
| `Algorithms/Landing/GroundClearanceTypes.hpp` | `ClearanceStatus`, flags, `GroundClearanceConfig`, `GroundClearance` |
| `Algorithms/Landing/GroundClearanceEstimator.hpp/.cpp` | LiDAR geometry, quality, closure rate, TTC |
| `Algorithms/Landing/LandingSafetyTypes.hpp` | `LandingWarning`, `WarningPolicy`, `LandingSafetyConfig`, `WarningState`, `SafetySnapshot` |
| `Algorithms/Landing/LandingSafetyAnalyzer.hpp/.cpp` | warning levels, persistence, hysteresis, cooldown, events |

All portable C++17 (no HAL, no FreeRTOS), fixed memory, no heap, no exceptions, bounded work.

### Preprocessing change (additive pass-through, like Milestone 4's `sAcc`)

* **What changed:** `LidarInfo` now carries `lastRejectReasons` and `lastRejectTimestampMs`, recorded in
  `PreprocessingPipeline::ingest()` when a LiDAR sample is **rejected**.
* **Why:** preprocessing keeps the last *accepted* sample, so after a rejection the frame only shows
  "invalid". Without the reason, "no return within 40 m" (normal at altitude) and "LiDAR dead" look the
  same, and only the first is evidence of being high.
* **Unchanged:** accepted-sample handling, validation and the driver.

---

## 3. Inputs and their validity

`SafetyInput` copies only what is used. The adapter maps the real types field by field:

| Sub-struct | Source fields (actual names) | Validity rule in the safety layer |
|---|---|---|
| `nav` | `NavigationEstimate::state, timestampMs, altitude_m, verticalVelocity_mps, *Valid, *Sigma*, qualityFlags` | fresh (≤ 500 ms), `velocityValid`, `velocitySigma_mps ≤ 1.0` |
| `attitude` | `AttitudeEstimate::q, orientationValid, state, timestampMs, qualityFlags, linearAccel.enu_mps2/valid/timestampMs` | `orientationValid`, no `GapDetected` / `NumericalFault`; LiDAR skew ≤ 40 ms |
| `imu` | `ProcessedSensorFrame::angularRate_radps.value/valid/timestampMs` | fresh |
| `lidar` | `ProcessedSensorFrame::lidar.distance_m, signalStrength, timestampMs, valid, lastRejectReasons, lastRejectTimestampMs` | new sample = new timestamp; age ≤ 60 ms |
| `gnss` | `gnss.velocityNorth/East_mps` (→ ground speed), `speedAccuracy_mps, fixType, numSatellites, timestampMs, valid` | 3-D fix, `sAcc ≤ 1 m/s`, age ≤ 2 s |

Every value has its own state. The safety layer distinguishes:
* valid;
* degraded (e.g. attitude Degraded → larger clearance σ; navigation Degraded → `NavigationDegraded`);
* unavailable;
* stale (age limits);
* ambiguous (flight phase `Unknown` / candidates → `safetyq::PhaseUncertain`);
* unverified hardware configuration (`LidarMountingUnverified`, `ImuMountingUnverified`).

**Missing is never zero:** invalid numbers are NaN with a status, in every output.

---

## 4. Flight-state detector

### 4.1 States

```
                  +-----------+  consistent evidence 5 s (or 1 airborne source 15 s)
                  |  Unknown  |------------------------------------------+
                  +-----------+                                          |
                        | >=2 ground sources, no airborne, 5 s           v
                        v                                           +---------+
   +--------+  any airborne source   +------------------+ >=2 src / LiDAR-high 3 s (no LiDAR veto) |
   | Ground |---------------------->| TakeoffCandidate |------------------------------------------>| Flying  |<--+
   +--------+<----------------------+------------------+  or 1 source (not altitude-departure) 15 s +---------+   |
       ^      no airborne evidence 2 s (aborted)  ^                                       | clearance<30 ^  |      |
       |                                          |                                       v      >45 /  |      |
   +--------+   any airborne source               |                                  +----------+ lost 3s      |
   | Landed |---------------------------------------+                                  | Approach |--------------+
   +--------+                                                                         +----------+
       ^   held 5 s                                        >=2 ground, 0 airborne, 2 s     |   (also from Flying)
       +------------------------------ LandingCandidate <----------------------------------+
                                        | airborne evidence / ground evidence lost 2 s -> back to Flying/Approach
```

`minPhaseDwellMs` (1 s) separates phase changes.

### 4.2 Evidence (independent sources, bit masks in `FlightStateStatus`)

| Airborne | Rule | Ground | Rule |
|---|---|---|---|
| `LidarHigh` (**strong**) | valid clearance > 2.5 m, or BeyondRange with lower bound > 2.5 m | `LidarNearGround` | valid clearance ≤ 2.5 m |
| `GnssFast` | ground speed > 6 m/s | `GnssSlow` | ground speed < 1 m/s |
| `VerticalMotion` | \|vz\| > 0.8 m/s for 3 s (valid nav) | `AltitudeSteady` | 8 s window: spread ≤ 2 m and net rate < 0.3 m/s |
| `AltitudeDeparture` | \|h − ground reference\| > 15 m (reference frozen at takeoff candidate) | `Still` | RMS \|linear accel\| < 0.4 m/s², RMS \|rate\| < 0.05 rad/s (τ = 2 s) |

Rules:

* **No single threshold decides.**
* **Altitude departure never confirms a takeoff alone.** Walking up a hill changes altitude.
* **A valid LiDAR near the ground vetoes takeoff confirmation.** Running at 7 m/s with the wing up, or
  a car ride, gives GNSS-fast but LiDAR says ~1 m. Result: the phase stays `TakeoffCandidate`, then
  aborts.
* **Landing needs ≥ 2 ground sources and no airborne evidence** (departure ignored after a flight),
  first for 2 s (`LandingCandidate`), then held for 5 s (`Landed`).
* **`AltitudeSteady` is judged over the whole window.** An early version tested the instantaneous
  \|vz\| and flickered with 0.12 m/s navigation noise. That kept restarting the dwell timers and
  delayed the navigation-only landing to 31.5 s (now 15.4 s; validation report §5).
* **No usable input at all** (navigation, LiDAR, GNSS and IMU all missing or stale): the phase is held
  (`PhaseHeld`) and every dwell timer is frozen. **A sensor outage never produces "Landed"**
  (A15: 20 s total outage in flight).
* **`support` (`EvidenceLevel`):** None / Weak / Moderate / Strong for the current phase. This is a
  count of agreeing sources, **not a probability**.
* **Ground reference altitude:** follows the UKF altitude while on the ground. It is frozen at the
  takeoff candidate and re-set at landing. It is evidence of launch / landing, **never** a ground
  clearance.

### 4.3 Situations covered (simulation)

Each case is a test in `test_flight_state.cpp`:
* normal hill launch;
* failed / aborted launch;
* device carried uphill;
* device picked up;
* device dropped;
* thermal climb at 25° bank;
* extended descent;
* low-altitude ridge soaring at low ground speed;
* approach, flare, touchdown;
* hard landing;
* stand still after landing;
* second takeoff;
* switched on in flight;
* navigation-only operation (no GNSS / LiDAR);
* 20 s total sensor outage.

---

## 5. Altitude vs clearance (why they are never mixed)

| Quantity | Reference | Used for |
|---|---|---|
| UKF `altitude_m` | the barometric calibration reference (launch) | launch / landing **evidence** (departure, steadiness); never terrain proximity |
| GNSS height | sea level / ellipsoid | not used |
| LiDAR clearance | the **illuminated terrain point** | terrain proximity, closure, TTC |

The ground under a paraglider changes all the time (hills, valleys, buildings). Takeoff altitude says
nothing about clearance away from the launch.

---

## 6. LiDAR ground clearance

### 6.1 Geometry

* **Beam:** a configured unit vector `b` in body FRD. Default `(0, 0, 1)` = down when level. This is a
  **placeholder**: `mountingVerified = false` and every output has `MountingUnverified`.
* **Beam in ENU:** `d_ENU = q ⊗ b ⊗ q*` with `q = q_ENU_from_body` (`quat::rotate`).
* **Projection factor:** `p = −d_ENU.z = cos(tilt from straight down)`.
* **Clearance:** `c = r · p`, where `r` is the TF02-Pro slant range.
* **Valid only if all of these hold:**
  * the LiDAR sample is new, accepted and ≤ 60 ms old;
  * the range is within 0.1–40 m;
  * the attitude is valid, not after an unconfirmed gyro gap, and ≤ 40 ms from the LiDAR sample;
  * `p ≥ cos(40°) = 0.766`.

  A horizontal or upward beam never gives a value (`BeamGeometry`).
* **`HighTilt`:** flags tilts over 20°. At that point the illuminated point is ≥ 0.36 r away
  horizontally.

**Uncertainty (1σ):**

```
σ_c² = (σ_r p)² + (r sin θ σ_θ)² + (½ r p σ_θ²)²
```

* σ_r: 6 cm below 6 m, 1 % above (TF02-Pro manual);
* σ_θ: attitude 2° (6° when degraded), ⊕ 5° while the mounting is unverified.
* σ_c does **not** include terrain slope, which is unknown to a single beam.

### 6.2 Terrain assumption (stated, quantified, not hidden)

`c` equals the height above the **illuminated point**. It equals the height above the ground
**beneath** only for locally horizontal terrain. The `FlatTerrainAssumed` flag is always set.

Simulated against independent ray–plane truth, 15 m above a slope:

| Slope | Bank 20° towards the slope | Bank 20° away from the slope |
|---|---|---|
| 10° | +1.03 m | −0.90 m |
| 20° | +2.29 m | −1.75 m |
| 30° | +3.99 m | −2.60 m |

With a vertical beam the error is 0. A single beam cannot reconstruct slope, so no correction is
attempted.

### 6.3 Quality and failures

| Situation | Status / flag | Behaviour |
|---|---|---|
| never any LiDAR data | `NoLidar` | NaN |
| same sample again | `NotNew` flag | previous result carried and ageing; not re-processed; counted |
| older than 60 ms | `Stale` | NaN, rate history reset |
| rejected: out of range | `BeyondRange` | NaN clearance; `lowerBound_m = 40 · p` when the geometry is usable |
| rejected: weak / saturated | `WeakSignal` | NaN |
| other rejection / out-of-range value | `InvalidMeasurement` | NaN |
| attitude invalid / gap | `AttitudeInvalid` | NaN |
| attitude ≥ 40 ms from the LiDAR | `AttitudeStale` | NaN (stale attitude never projects fresh LiDAR) |
| beam > 40° from down | `BeamGeometry` | NaN |
| attitude Degraded / Recovering | `AttitudeDegraded` flag | valid, larger σ |
| jump > max(1.5 m, 10 %) vs prediction | `StepChange` flag | **kept valid** (cliffs, buildings, vegetation are real); rate history restarted |

**No smoothing of the clearance itself:** a 12 m cliff drop is reported in the next sample (validation
§6.4). A single spike only invalidates the rate briefly (step flag + residual check).

### 6.4 Closure rate

* **Fit:** least-squares slope of the clearance over the last 400 ms, with timestamps (variable dt,
  wrap-safe). At least 6 samples spanning ≥ 200 ms since the last reset (step, stale, unavailable).
  Never from one sample.
* **Output:** `closureRate_mps = −slope` (**positive = approaching terrain**). σ comes from the
  regression, never below the per-sample clearance σ.
* **Rejection:** an RMS residual > 0.5 m (spiky or strongly curved) gives no rate.
* **Latency:** about half the window (≤ 200 ms), reported in `rateLatencyMs`.
* **What it measures:** the rate relative to the **illuminated surface**. It is not the UKF vertical
  speed. Terrain rising under level flight gives closure with vz = 0 (validation §6.3: 1.99 m/s
  measured vs 1.8 true). Range noise of 1 % at 30 m gives about 0.3–0.4 m/s rate noise, and the
  reported σ describes it.

### 6.5 Time to contact

* **Formula:** `TTC = c / closure`, constant-closure approximation.
* **Validity:** valid only if the clearance and rate are valid, closure > 0.3 m/s, closure − 2σ > 0
  (significantly approaching) and TTC ≤ 60 s. Otherwise `ttcValid = false` and `timeToContact_s = NaN`
  ("no contact predicted"; never 0, never infinity).
* **No acceleration term:** a vertical-acceleration term was not added. A second-order TTC needs the
  acceleration **relative to the terrain**, which the UKF's vertical acceleration is not on uneven
  terrain.
* **Assumptions:** constant closure and flat terrain. TTC is an approximation, not a guaranteed impact
  time.

---

## 7. Safety events

### 7.1 SafetyEvent (40 bytes, trivially copyable)

* **Identity:** `eventId` (increasing; a gap = an event lost downstream), `episodeId` (one per warning
  activation), `timestampMs`.
* **Classification:** `type`, `severity`, `previousSeverity`, `transition`, `source`, `phase`.
* **Repeats and quality:** `suppressedCount`, `qualityFlags`.
* **Values at the decision** (NaN if n/a): vertical velocity, clearance, closure rate, TTC.

Types:

| Type | Meaning |
|---|---|
| `FlightPhaseChanged` | Notice |
| `ExcessiveDescent` | navigation vertical speed beyond the descent thresholds |
| `LowGroundClearance` | LiDAR clearance low |
| `RapidGroundClosure` | LiDAR closure rate / time to contact |
| `HazardousApproach` | low clearance and fast descent / closure |
| `TerrainClearanceUnavailable` | airborne near terrain, or descending fast, without usable LiDAR |
| `NavigationDegraded` | UKF degraded / unavailable / velocity too uncertain while airborne |
| `CrashSuspected`, `EmergencyRequest` | **reserved** for the next packages |

**Severity:** Info < Advisory < Caution < Warning < Critical.

**Transitions:**

| Transition | Meaning |
|---|---|
| `Raised` | new warning, new episode |
| `Escalated` | same episode, higher severity |
| `Deescalated` | same episode, lower severity |
| `Cleared` | same episode, condition gone |
| `Notice` | one-shot |

**Events are transitions only.** A warning held for 60 s produces exactly one event (test
`ls_events_only_on_transitions`). "Still active" is read from the snapshot, never repeated in the queue.

### 7.2 Queue and overflow policy (`SafetyEventQueue<N>`, N = 16 in firmware)

* **Normal operation:** FIFO, one producer, one consumer, `ICriticalSection` (RTOS critical section in
  firmware).
* **When full:** if a queued event has a **lower** severity than the new one, the oldest of the
  lowest-severity events is dropped. Otherwise:
  * a new Critical drops the oldest queued event (counted in `droppedCritical` if that was also
    Critical);
  * anything else is itself dropped (`droppedNew`).
* **Guarantee:** a critical event is **never** lost because the queue is full of lower-priority
  messages. Test: 2000 mixed events and a consumer 10× slower lose 0 critical events.
* **Firmware note:** no consumer task exists yet, so the firmware queue fills with the first 16 events
  and then keeps the most severe ones. Health shows `safetyQueueDepth/HighWater/Dropped*`. The future
  audio / logging task pops it. A second consumer needs a second queue registered with
  `SafetyMonitor::addSink` (up to 4).

### 7.3 Snapshot (`SafetySnapshot`, 188 B)

* phase and support, evidence bits;
* navigation summary;
* clearance / rate / TTC;
* per-warning `WarningState` (active, severity, announced, degraded, episode, since);
* `highestSeverity` and `activeWarningMask`;
* `safetyq` flags (always `Unvalidated`; `ModelPlaceholder` propagated from the UKF);
* counters.

Published every frame through `SafetySnapshotMailbox`.

---

## 8. Landing safety analyzer

### 8.1 Warning levels and evidence (provisional values)

| Warning | Evidence needed | Levels (entry; exit = hysteresis) | Phases |
|---|---|---|---|
| `ExcessiveDescent` | navigation vz (fresh, valid, σ ≤ 1 m/s) | Caution 3.5, Warning 6, Critical 10 m/s sink (exit −0.5) | all except Ground / Landed |
| `LowGroundClearance` | LiDAR clearance | Advisory < 10 m (exit > 15 m) | Flying, Approach, Unknown (Unknown only above 2.5 m: a device standing on the ground is not "low") |
| `RapidGroundClosure` | LiDAR clearance + closure rate / TTC only (works with no navigation) | Caution closure > 4 m/s; Warning TTC < 4 s; Critical TTC < 2 s (clearance ≤ 40 m) | airborne + Unknown |
| `HazardousApproach` | LiDAR clearance + (LiDAR closure, or −vz from navigation = **degraded evidence**) | Warning < 15 m and > 2.5 m/s; Critical < 6 m and > 3.5 m/s | airborne + Unknown |
| `TerrainClearanceUnavailable` | LiDAR status + (recent clearance < 45 m within 30 s, or Approach, or sink > 1.5 m/s) | Advisory; Caution if sink > 3.5 m/s | Flying, Approach, LandingCandidate |
| `NavigationDegraded` | navigation state / velocity σ | Advisory: Degraded / Recovering / velocity σ > 1 m/s; Caution: Unavailable / fault / not initialised | airborne |

**Which warnings depend on trustworthy navigation:** `ExcessiveDescent` (fully), `HazardousApproach`
(only as a fallback, flagged degraded), and `TerrainClearanceUnavailable` (only for its Caution level).
`LowGroundClearance` and `RapidGroundClosure` are **LiDAR-only** and work with the UKF unavailable. The
UKF's small covariance is not trusted blindly: velocity is used only fresh and with σ ≤ 1 m/s, and every
event carries `ModelPlaceholder`.

### 8.2 Persistence, hysteresis, cooldown, escalation (`WarningPolicy`)

| Warning | raise | escalate | relax (de-escalate / clear) | cooldown |
|---|---|---|---|---|
| ExcessiveDescent | 1.0 s | 0.5 s | 2.0 s | 20 s |
| LowGroundClearance | 1.0 s | – | 2.0 s | 30 s |
| RapidGroundClosure | 0.3 s | 0.2 s | 1.0 s | 10 s |
| HazardousApproach | 0.5 s | 0.3 s | 1.5 s | 10 s |
| TerrainClearanceUnavailable | 3 s unusable (in condition) | 1 s | 2 s | 60 s |
| NavigationDegraded | 2 s | 1 s | 3 s | 60 s |

* **Critical is faster:** it is raised after half the raise time and escalated without delay.
* **Cooldown:** a re-activation at a severity no higher than the one last cleared is tracked (visible in
  the snapshot), but **not announced** (`suppressedReactivations`). Anything more severe, and any
  Critical, is announced at once.
* **Escalation is never suppressed.**
* **Missing evidence is not "safe":** when the input a warning needs disappears, the warning is held at
  its level for `evidenceHoldMs` (3 s) and flagged degraded, then released.
* **Ground / Landed:** all warnings are released, so there are no repetitive post-landing alarms.
* **Unknown phase:** warnings are **not** suppressed; events carry `PhaseUncertain`.

### 8.3 Unknown phase and conflicting observations

* **Unknown phase:** all descent / closure warnings are evaluated (starting in flight must not silence
  them).
* **Navigation vs LiDAR:** the LiDAR closure rate takes precedence over navigation vz for
  terrain-relative hazards. Example: navigation says −6 m/s (e.g. a barometer fault) while the LiDAR
  shows a steady 20 m. `ExcessiveDescent` (navigation-based) is raised, but no terrain hazard is
  confirmed (test `integ_faults_contradictory_...`).
* **LiDAR unusable:** terrain hazards are not confirmed. `TerrainClearanceUnavailable` explains the
  gap, and `HazardousApproach` may still be raised from navigation descent, flagged
  `DegradedEvidence`.

### 8.4 Normal vs risky situations (simulation)

* **Normal** (no Caution or higher in 20 normal flights over 10 seeds): cruise, thermal climb, glide,
  approach, flare, touchdown, standing. The only advisory is `LowGroundClearance` on final approach,
  informational.
* **Risky:**

  | Situation | Result |
  |---|---|
  | B-stall-like −7 m/s | ExcessiveDescent Warning after 1.0 s |
  | spiral −12 m/s | Critical after 0.01 s, as an escalation |
  | level flight into terrain rising 0.6 | RapidClosure Critical 1 s before the ground is 6 m away |
  | −5 m/s without LiDAR | TerrainClearanceUnavailable Caution |
  | LiDAR dies on final | Advisory after 3 s, no invented hazard |
  | uncertain navigation velocity | no descent warning; `NavigationDegraded` explains why |

### 8.5 Repetition

Each warning type has one tracker, and events come only from tracker transitions. A permanently
unavailable sensor produces **one** Raised event (e.g. `TerrainClearanceUnavailable` in C34), never
periodic urgent alerts. There is no escalation policy for an unavailable sensor; it stays at its level.

### 8.6 Flare

**Not implemented.** Flare timing depends on the wing, airspeed, wind, technique, terrain and sensor
position. A single downward beam plus a vertical-speed estimate does not determine it, and no
prescriptive flight-control indication is produced. A future optional indication must be disabled by
default and labelled unvalidated.

---

## 9. FreeRTOS integration

* **Placement:** one line in `SensorProcessingTask::Run()`, after navigation:
  ```cpp
  safetyOut_.publish(safety_.update(pg::safetyInputFrom(work_, attitude_.latest(), navigation_.latest())));
  ```
* **What does not change:** no new task, queue or priority. Acquisition is 40, processing 32 and health
  16. Sampling rates and queue policies are unchanged.
* **Ownership:** `SafetyMonitor`, the snapshot mailbox and the event queue are members of the
  processing task (static, constructed before the scheduler). The queue sink is registered in the
  constructor.
* **Start-up check:** `SOAR_ASSERT(safety_.configValid())` in `InitTask`.
* **Accessors:** `SensorProcessingTask::safety()` (snapshot mailbox, thread-safe copy) and
  `safetyEvents()` (queue; one consumer pops).
* **Health (`g_systemHealth`, 24 fields):**
  * phase, support, clearance status / cm, closure (mm/s), TTC (ms), active mask, highest severity,
    quality flags;
  * counters: transitions, takeoffs, landings, warnings raised, low-clearance raised, suppressed,
    LiDAR rejected, stale clearance, events published;
  * queue depth, high water and the three drop counters.
* **Not available:** maximum safety-processing time. The task's processing time has only 1 ms
  resolution; a DWT cycle measurement is in the hardware TODO.

### Resources (Release `-Os`, CubeIDE)

| Item | Value |
|---|---|
| Flash | +11 584 B text total (safety code 9 500 B; the rest is integration and health) |
| Static RAM | +3 088 B bss: SensorProcessingTask object 18 352 → 21 160 B (SafetyMonitor 1 896, queue 680, snapshot mailbox 196, locks); HealthMonitorTask +196 B (snapshot copy); `g_systemHealth` +84 B |
| Heap | 0 |
| Stack | `Run` 256 → 296 B; safety chain `SafetyMonitor::update` 136 + `LandingSafetyAnalyzer::update` 168 + `step` 104 ≈ 0.4 KB (after navigation returns). The deepest chain is still navigation: 296 + 672 = 968 B of 4 096 B |
| Host time | 126 ns per `SafetyMonitor::update` (Apple M4, `-O2`) |
| Target time | **estimate** 40–130 µs per update at 64 MHz (Milestone 3 host→target ratio); not measured |
| Critical sections | snapshot copy 188 B; event copy 40 B per push (a few µs, estimate) |

---

## 10. Future crash detection and SOS

* **Producer:** the crash detector produces `SafetyEvent{type = CrashSuspected, source = CrashDetector}`
  through the same emitter (shared numbering).
* **Emergency manager:** consumes the queue and snapshot. It decides on SOS from `CrashSuspected` +
  `phase` + `Landed` / impact evidence + time without movement.
* **Audio / LED:** play `Raised` / `Escalated` once; repeat from the snapshot while `active`; stop on
  `Cleared`; respect `severity` priority. This needs no change to the producers.
* **Logging:** pops a second queue (`addSink`); `eventId` gaps reveal losses.

---

## 11. Known limitations

1. **Simulation only.** Synthetic scenarios are classes of motion, not paraglider recordings. Every
   threshold is provisional.
2. **LiDAR mounting is unverified.** A 10° mounting error gives up to 0.85 m error at 10 m (simulated).
3. **Flat-terrain assumption:** errors up to ±4 m at 15 m over a 30° slope with 20° bank. The beam may
   illuminate obstacles or terrain ahead.
4. **Closure-rate noise** of about 0.3–0.4 m/s at 30 m range (1 % range noise) with a 200 ms latency.
5. **Flight-phase heuristics:**
   * Without LiDAR and GNSS, phases rely on navigation + IMU stillness: slower decisions (takeoff
     16.5 s, landing 15.4 s), and very calm level flight could look "steady + still".
   * A car ride with the LiDAR not seeing the ground could look like flight.
6. **UKF caveats inherited:** accelerometer-bias sensitivity and over-confident altitude σ.
   ExcessiveDescent uses navigation velocity and its σ gate, not altitude σ.
7. **Single queue consumer;** no consumer exists yet in firmware.
8. **Not measured:** target timing and stack high-water mark.
