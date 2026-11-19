# Crash Detection, Emergency Manager, Safety-Event Dispatcher and Alert Outputs: Design (Milestone 6)

**Status: implemented and simulation-validated only.** Nothing has run on hardware. No threshold,
countdown or retry parameter is validated. A *suspected crash* is not a confirmed crash, and the
firmware cannot tell whether anyone is injured. The SOS transport, audio, LED and persistence backends
in firmware are explicitly *unconfigured*. Nothing here is flight-ready, safety-certified or approved
for contacting emergency services.

Companion documents:
- `CRASH_EMERGENCY_VALIDATION_REPORT.md`: results.
- `CRASH_EMERGENCY_WALKTHROUGH.txt`: line-by-line explanation for review.
- `CRASH_EMERGENCY_GIT_SUMMARY.md`: changes.

---

## 1. What this milestone adds

```
SensorAcq --> SensorProc: preprocessing -> Madgwick -> UKF -> SafetyMonitor -> CrashDetector  (per 10 ms frame)
                                                          |              |
                                   SafetyEventQueue<16> <-+--------------+   (transitions: landing + crash)
                                   SafetySnapshotMailbox, CrashReportMailbox  (latest values)
                                                          |
EmergencyAlertTask (every 20 ms or on a command):         v
   commands / transport reports -> EmergencyManager -> ISosTransport  (UNCONFIGURED in firmware)
                                        |  EmergencyStatus, PendingSosRecord (latest values)
   events + snapshot + crash + emergency -> SafetyEventDispatcher -> AudioAlertScheduler -> IAudioAlert (UNCONFIGURED)
                                                                   -> LedPatternScheduler -> IStatusLed  (UNCONFIGURED)
```

There are four independently testable parts:

| Part | Where | Job |
|---|---|---|
| A. CrashDetector | `Components/Algorithms/Crash/` | Turns observations into *possible impact* and *suspected crash* |
| B. EmergencyManager | `Components/Algorithms/Emergency/` | Runs an incident: countdown, cancel, manual SOS, request, retries, acknowledgments, outcome, re-arm |
| C. SafetyEventDispatcher | `Components/Algorithms/Safety/` | The missing consumer of the Milestone 5 event queue: active alerts, priorities, acknowledgment |
| D. Output schedulers | `Components/Output/Audio/`, `Components/Output/LED/` | Non-blocking pattern timing over `IAudioAlert` and `IStatusLed` |

None of these call HAL, GPIO, UART, Bluetooth or speaker APIs. All are portable C++17 with fixed memory
and are unit-tested on a laptop.

## 2. Terminology (deliberate)

| Term | Meaning |
|---|---|
| Impact candidate | A corroborated, impulsive IMU peak. A *possible* impact. |
| Suspected crash | An impact candidate plus enough independent supporting evidence under an explicit rule. This is a request to *ask the pilot*, not a finding. |
| Incident | The EmergencyManager's episode: suspicion or manual SOS → countdown → request → … |
| SOS request created / left the device / accepted by phone / delivery reported | Four different facts (section 10). One is never reported as another. |

## 3. Inputs and why they are taken *before* smoothing

### 3.1 Impulse data: per-IMU peaks

`ProcessedSensorFrame::imu[i]` gained six additive fields, filled in `PreprocessingPipeline::handleImu`:
- `peakAccelNorm_mps2`
- `peakGyroNorm_radps`
- `peakTimestampMs`
- `samplesSinceFrame`
- `accelSaturatedSinceFrame`
- `gyroSaturatedSinceFrame`

They hold the **largest** |specific force| and |rate| of all samples ingested since the previous frame.
Without this, a delayed batch would expose only the newest sample.

The fields are filled from validated, factory-calibrated, sensor-axis values:
- **before** the redundancy voter (which excludes a saturated channel);
- **before** Madgwick and the UKF (which smooth and gate);
- **including** saturated and out-of-range samples. Their value is a *lower bound* and is flagged.

The UKF is used only for *context*: the sink rate before the impact. It would attenuate, gate or
reject the impulse itself.

### 3.2 Other inputs

`CrashInput` (124 B on ARM) is filled by `crashInputFrom(frame, attitude, snapshot, navigation)`:
- the voted |f| and |ω| (for quiet and free-fall checks);
- attitude tilt (the angle of the body z axis from the vertical, through `quat::rotate`);
- flight phase, LiDAR clearance and closure rate from the `SafetySnapshot` of the same frame;
- navigation vz;
- GNSS ground speed with its timestamp.

Each source keeps its own validity. Invalid values never become 0.

### 3.3 Sampling limits (documented, not solved)

- The LSM6DSRX runs at 416 Hz ODR but is **polled at 100 Hz**. Each poll reads the newest output, so
  an impact pulse shorter than ~10 ms can be **missed or under-read**. The simulator models this:
  each IMU is sampled at its own instant inside the cycle. The on-chip digital filter (bandwidth set by
  ODR and LPF settings) also attenuates very short pulses. Its configuration was not chosen for impacts.
- Full scale is ±16 g / ±2000 dps. Harder impacts **saturate**: the detector sees a flagged lower
  bound, and a saturated pulse counts as "strong" when corroborated.
- Recommended hardware follow-ups (HARDWARE_INTEGRATION_TODO.md §16): FIFO reading of all samples,
  the chip's wake-up / free-fall engine, and a deliberate filter choice.

### 3.4 Not gravity-free

The impulse uses |specific force|, which is about 1 g at rest and near 0 g in free fall. It is **not**
linear acceleration, and gravity is not subtracted anywhere in the detector, so it can't be subtracted
twice. "Quiet" means ||f| − g| small, which needs no attitude.

## 4. Corroboration: one IMU can neither create nor hide an impact

`readImpulse()` sorts the per-IMU peaks and uses the **second-largest** as the impulse value
("corroborated"):

| IMUs with data | Rule | Effect |
|---|---|---|
| 3 | 2nd largest | One faulty IMU, whether spiking high or stuck low, is outvoted. |
| 2 | Smaller | Conservative: a single spike is ignored. |
| 1 | That IMU | The class is capped at *Moderate*, flagged `SingleImuOnly`. |

A high peak seen by only one IMU is counted in `uncorroboratedPeaks`. Saturation counts when ≥ 2 IMUs
saturate, or when the only IMU does. Repeated or older peak timestamps are rejected per IMU, so each
sample is evaluated once.

## 5. Evidence and initial thresholds (provisional)

| Parameter | Default | Why this initial value |
|---|---|---|
| `impactAccel` | 4 g | Above turbulence (1.5–2.5 g) and normal landings (1.5–3 g). Typical hard-impact pulses are 5–20 g. |
| `impulseMin` (above the EMA baseline, τ 0.5 s) | 2 g | A spiral *sustains* 3–4 g. Only a sudden rise is an impulse. |
| `severeAccel` | 8 g | "Strong" without further IMU evidence. |
| `impactRate` / `rateImpulseMin` | 400 / 250 dps above baseline | Spirals rotate at ~100–200 dps. Tumbling exceeds 400. Added after the synthetic spiral produced a false suspicion (validation report §3.2). |
| `severeRate` | 800 dps | Strong rotation impulse. |
| Strong class | ≥ 8 g, or ≥ 800 dps impulse, or ≥ 4 g with a rate impulse, or corroborated saturation | Never from one IMU. |
| `candidateWindow` | 500 ms | Collects the rest of one pulse (multi-frame). |
| `preWindow` | 3 s | "Just before the impact". |
| `recentAirborne` | 30 s | Landed, then incident, still eligible. Ground and Landed don't override a recent flight. |
| `fastDescent` | 4 m/s (UKF) | Trim sink is ~1–1.5 m/s and big ears ~3 m/s. |
| `GroundClosure` | LiDAR ≤ 10 m and closing ≥ 3 m/s | Terrain approaching fast. |
| `highGroundSpeed` | 13 m/s | Above trim ground speed in calm air. |
| Free fall | \|f\| < 0.35 g for ≥ 150 ms, ending ≤ 300 ms before the impact | A dropped device. |
| `minObservation` / `observation` | 2 s / 10 s | Earliest and latest decision. |
| Quiet | \|\|f\|−g\| < 0.8 m/s² and \|ω\| < 0.2 rad/s; blips ≤ 200 ms tolerated | Lying still. |
| `inactivity` | 4 s quiet | Post-impact support. |
| Abnormal orientation | tilt ≥ 60° while \|ω\| < 0.5 rad/s for ≥ 2 s | Lying, not a banked turn. |
| Repeated impacts | ≥ 2 further pulses (≥ 300 ms apart) | Tumbling. |
| Pilot activity | \|\|f\|−g\| > 1.5 m/s² for ≥ 3 s cumulative | *Reported*, not counter-evidence. |
| Resumed flight | GNSS ≥ 6 m/s for 3 s, phase airborne, LiDAR clearance > 3 m (if valid) | Shorter than inactivity, so a turbulence hit in calm air is recognised as flight first. |

Evidence bits (`crashev::*`) and quality flags (`crashflag::*`) are reported in `CrashStatus`, in the
`SafetyEvent`s and in `g_systemHealth`. These are observations, not proof.

## 6. Crash detector state machine

```
             impulse                 +500 ms (pulse collected)
 Monitoring ---------> ImpactCandidate -------------------------> PostImpactObservation
    ^  ^                     | not eligible (never airborne / drop)      |  rule satisfied (>= 2 s)
    |  |                     v                                            v
    |  +---- 1 s ------- Rejected <---- window end / flight resumed --- CrashSuspected
    |                                                                     | flight resumed (Cleared event)
    +------------------ 30 s --------- Cooldown <-------- 60 s hold ------+
 Unavailable <--> Monitoring : no IMU data for 1 s / data back
```

### 6.1 Eligibility (at the end of the candidate window)

**Eligible** means any of:
- airborne in the pre-window;
- a TakeoffCandidate in the pre-window;
- airborne in the last 30 s;
- phase Unknown with a strong impulse (device started in flight).

Otherwise the candidate is **Rejected**:
- `NoFlightContext` (bumps or handling on the ground);
- `DropSignature` (free fall then impact without flight context).

### 6.2 Pre-impact count (from the bounded history)

The pre-impact count is the number of these indicators present:
- AirborneAtImpact, or TakeoffAttempt (not both);
- FastDescent;
- GroundClosure;
- HighGroundSpeed;
- FreeFallBeforeImpact (counted only with flight context).

### 6.3 Post-impact count

The post-impact count is the number of these indicators present:
- Inactivity;
- AbnormalOrientation;
- RepeatedImpacts.

PilotActivity is reported but never subtracted, because a conscious pilot may move.

### 6.4 Decision rule (evaluated from 2 s after the impact)

| Impulse | Suspect when | Decision code |
|---|---|---|
| Strong | pre ≥ 2 | `StrongImpactWithContext` |
| Strong | pre ≥ 1 and post ≥ 1 | `StrongImpactWithSupport` |
| Strong | post ≥ 2 | `ImpactWithPostSupport` |
| Moderate | (pre ≥ 2 and post ≥ 1) or (pre ≥ 1 and post ≥ 2) | `ImpactWithPostSupport` |
| Strong, airborne at impact, at window end (10 s), flight not resumed | always | `StrongImpactFlightEnded`, or `StrongImpactDataLoss` if IMU data were missing for ≥ 50 % of the window |
| Anything, flight resumed | never | Rejected `FlightResumed` |
| Otherwise, at window end | never | Rejected `InsufficientEvidence` |

So **a single accelerometer threshold can never produce a suspicion**. A weak isolated spike returns to
Monitoring.

### 6.5 Missing data (explicit policy)

- Missing post-impact data is **not inactivity** and **not proof of safety**. A strong in-flight impact
  whose data then vanish is decided on the pre-impact evidence at the window end (`StrongImpactDataLoss`,
  configurable `suspectOnPostImpactDataLoss`).
- If the **whole processing task** stops after a strong in-flight impact, no further reports arrive. The
  EmergencyManager opens an incident after `detectorSilenceMs` = 5 s (`IncidentTrigger::DetectorSilence`,
  once per crash episode).

### 6.6 Re-arm and repeated impacts

- CrashSuspected is held 60 s; the manager latched it long before.
- Cooldown then lasts 30 s. During it, further impulses are counted in `impact.impacts` and no new
  suspicion opens.
- Rejected is shown for 1 s, then Monitoring.
- A suspicion is cleared (event `CrashSuspected/Cleared`) only when flight resumes.

### 6.7 Time and outages

- Every interval is a wrap-safe `uint32` difference of real timestamps, never a frame count.
- Frames older than the previous one are ignored.
- A gap longer than 2 history slots sets `HistoryGap` on the next episode. A gap longer than the
  8 s history clears it.
- **What survives an outage:**
  - the detector state and episode;
  - the impact summary;
  - the last-airborne time;
  - the evidence timers (bounded: one gap counts for at most one observation window).
- **What does not survive:** history older than 8 s. The context it held is not "just before the
  impact" any more.

## 7. EmergencyManager state machine

| State | Entered when | Leaves to |
|---|---|---|
| Idle | Start, reset, or re-armed | Countdown (suspicion, manual SOS, detector silence); SosPending if the countdown is 0 |
| Countdown | Incident opened | SosPending (strictly after the deadline, or a manual SOS confirms); Cancelled |
| SosPending | Request created; or an attempt failed (backoff); or link lost | AwaitingPhoneAck (transport accepted the request) |
| AwaitingPhoneAck | Transport accepted | AwaitingDelivery (PhoneAccepted / PhoneSendAttempted); SosPending (timeout / write failed / phone rejected / link down); DeliveryFailed (attempts exhausted) |
| AwaitingDelivery | Phone accepted | Delivered; DeliveryFailed; DeliveryUnknown (120 s without outcome) |
| Delivered | Phone **reported** confirmation | Idle (ResetIncident); SosPending (manual SOS re-sends with a newer position) |
| DeliveryUnknown | Accepted, no outcome in time | Delivered / DeliveryFailed (late report); Idle (reset); re-send (manual SOS) |
| DeliveryFailed | Phone reported failure, or no acceptance after `maxAttempts` | SosPending (reconnect: one more cycle, at most `maxCycles` = 3; or manual SOS); Idle (reset) |
| Cancelled | CancelSos for this incident, or suspicion cleared during the countdown | Idle after 10 s, once any cancellation notice is resolved |
| Unavailable | Invalid configuration | — |

Incident ids:
- They start at 1 and are never 0.
- With persistence they continue across resets.
- Without persistence (today) they restart at 1, flagged `IncidentIdVolatile` in the request.

A new suspicion **during** an incident increments `additionalImpacts`; it never opens a second
incident. A suspicion that the detector cleared (flight resumed) during an *automatic* countdown
cancels it (`CancelReason::SuspicionCleared`) unless the pilot confirmed it manually.

## 8. Countdown and cancellation

- The countdown is a timestamp deadline (`now + 60 s` automatic, `+ 5 s` manual; placeholders). It is
  advanced only by `tick(now)`, never by update counts. It is non-blocking, with no HAL_Delay or long
  RTOS delays.
- **Boundary rule:** a CancelSos *issued at or before* the deadline is in time. Expiry happens strictly
  after the deadline. The task applies commands before ticking, so a cancel received before a wake
  always beats an expiry in that wake. The host test shows the request is created exactly 60 001 ms
  after the start.
- **Scheduler gaps:** a late tick still expires the countdown. The lateness goes into
  `countdownOverrunMs` and the `CountdownOverrun` flag. A cancel *issued* before the deadline but
  processed after a gap is still honoured.
- **Cancellation is incident-specific:** it needs the incident id. A command with incident id 0 is
  malformed, and a wrong id is rejected (`WrongIncident`).
- **After the request has left the device**, cancellation **cannot withdraw** what the phone may
  already have sent. The manager:
  - sets `cancelledAfterSend`;
  - sends a cancellation *notice* (`SosRequestKind::Cancel`, same incident id, next revision) with the
    same retry machinery;
  - reports `CancelNotice` Pending, then Sent, then Accepted or Failed. *Accepted* only means the phone
    accepted the notice, not that any contact was told.

## 9. Manual SOS and commands

`EmergencyCommand` (16 B): type, source, per-source sequence, incident id, issue time, integrity word.

- It must be built with `makeEmergencyCommand()`. A zeroed, truncated or bit-flipped command fails
  `isWellFormed()` and is rejected: **no malformed command creates an SOS**.
- No buttons are assumed. Sources are `HardwareInput`, `PhoneApp`, `Diagnostic` and `Test`.
- The same (source, sequence) twice is a duplicate and is ignored.

| Command | Effect |
|---|---|
| ManualSos (id 0) | Idle or Cancelled: new incident (no crash evidence needed). Countdown: confirm now. Sending: no effect (idempotent). Delivered / Unknown / Failed: re-send the same incident with a fresh position. |
| CancelSos (id required) | Section 8. |
| AcknowledgeAlert | Forwarded to the dispatcher: silences an acknowledgeable alert episode until it escalates. Never the countdown or crash alarm. |
| RequestStatus | Accepted. The status is published every wake anyway. |
| ResetIncident (id required) | Delivered / Unknown / Failed / Cancelled → Idle. While counting down or sending: `NotAllowed` (use CancelSos). |

## 10. SOS request, delivery stages, acknowledgment and retries

### 10.1 Format (v1, 64 bytes, little-endian, CRC-32/IEEE over bytes 0–59)

| Offset | Field | Offset | Field |
|---|---|---|---|
| 0 | version (=1) | 34 | fixType |
| 1 | kind (1 alert, 2 cancel) | 35 | numSatellites |
| 2 | trigger | 36 | fixAgeMs (0xFFFFFFFF = never a fix) |
| 3 | emergency state | 40 | relativeAltitude_dm (baro, NOT MSL) |
| 4 | incidentId | 44 | verticalSpeed_cmps |
| 8 | revision | 46 | peakAccel_cg (0.01 g) |
| 10 | validity bits | 48 | peakRate_dps |
| 12 | createdUptimeMs | 50 | impactClass |
| 16 | eventUptimeMs | 51 | impacts |
| 20 | latitude_1e7 | 52 | crashEvidence bits |
| 24 | longitude_1e7 | 56 | battery_mV (0 = unknown) |
| 28 | heightMsl_dm (GNSS) | 58 | device health (low 16 quality bits) |
| 32 | hAcc_dm | 60 | CRC-32 |

Content rules:
- **Position** is the *last valid* fix with its age. The validity bit is clear if there never was one,
  and the coordinates are then meaningless.
- `PositionStale` is set when the fix is older than 60 s.
- GNSS MSL height and the barometric relative altitude are separate fields and are never substituted
  for each other.
- Navigation values older than 5 s are not sent.
- There is no personal data.
- `toLegacySosPayload()` maps a request onto the existing unversioned `SosPayload` (which is kept
  unchanged), using MSL height only.
- Unknown versions and corrupted frames are rejected by `decodeSosRequest()`. A test flips every
  single bit of the frame.

### 10.2 Stages (never conflated)

```
Created -> QueuedForTransport -> TransferredToPhone -> AcceptedByPhone -> PhoneSendAttempted -> DeliveryConfirmed
   firmware   transport submit()   BLE write done        phone app           phone app            phone app REPORT
```

| Stage | Reported by |
|---|---|
| Created | Firmware |
| QueuedForTransport | Transport `submit()` |
| TransferredToPhone | BLE write done |
| AcceptedByPhone | Phone app |
| PhoneSendAttempted | Phone app |
| DeliveryConfirmed | Phone app *report* |

Rules:
- `reportedStages` records only the stages that were **actually reported** (provenance).
- A later stage reported without the earlier ones advances the stage but sets `ProvenanceIncomplete`,
  and the earlier bit stays clear.
- Delivered means "the phone app *reported* a confirmation". The firmware cannot verify it, and an SMS
  "delivered" report is not a person reading it.
- DeliveryUnknown and DeliveryFailed are explicit outcomes that are never turned into success.
- A failure report after a confirmation is counted and ignored.

### 10.3 Correlation and idempotency

Every report carries (incident id, request revision).
- A report for another incident is `wrongIncidentReports`.
- A report for a revision never sent is `invalidReports`.
- A repeated stage is `duplicateAcks` and has no effect. In particular, a duplicate acknowledgment can
  never create an incident.
- A late acceptance of an older revision of the *same* incident is valid.

### 10.4 Retry policy (bounded)

- An attempt is a `submit()` the transport *accepted*.
- If there is no phone acceptance within 10 s, the attempt failed. The backoff is 2, 4, 8, 16, 32 s,
  then 60 s.
- After 8 attempts the state is `DeliveryFailed / RetriesExhausted`.
- A reconnect (LinkUp) starts one more cycle (at most 3). A manual SOS always re-sends.
- **Waiting for a link is not an attempt.** A request whose transport is down stays pending
  indefinitely, with `TransportUnavailable` raised; it never silently fails.
- Busy transport: retry after 250 ms, not counted.

### 10.5 Persistence (interface + mock only)

`IEmergencyStore` stores a 32-byte record:
- magic "PGER", version;
- state, trigger, stage;
- next incident id, incident id;
- revision, reported stages;
- flags, cancel notice;
- CRC-32.

The manager writes it on transitions, never per tick. `restore()`:
- **Countdown at reset:** restarts the *full* countdown.
- **Request in flight:** re-sends it (same id, next revision, `Restored` bit).
- **Delivered or Failed:** kept until a reset command.
- **Corrupt record:** ignored.

**The firmware uses `NullEmergencyStore`**, so `PersistenceUnavailable | IncidentIdVolatile` is
reported. **SOS continuation across an MCU reset is currently NOT possible.** A W25Q128 backend (two
alternating records, written asynchronously from a low-priority task) belongs to the logging milestone.

### 10.6 The reliable pending-request record

The `SosRequest` lives in the manager and is published every wake as `PendingSosRecord` (latest value)
by `EmergencyAlertTask::sosRequest()`. It does not depend on the event queue. **An overflowing
advisory queue cannot erase a pending SOS.**

## 11. Safety events, dispatcher and alerts

### 11.1 Events (reusing the Milestone 5 `SafetyEvent`, 40 B)

New types fill the reserved range. They follow the existing *condition + transition* convention.

| Type | Transitions | Source / episodeId |
|---|---|---|
| ImpactCandidate (34) | Raised (Advisory) / Cleared | CrashDetector / crash episode |
| CrashSuspected (32) | Raised (Critical) / Cleared (flight resumed) | CrashDetector / crash episode |
| EmergencyCountdown (35) | Raised (Critical) / Cleared | Emergency / incident id |
| EmergencyCancelled (36) | Notice | Emergency / incident id |
| EmergencyRequest (33) | Notice (Critical) — request created | Emergency / incident id |
| SosTransportUnavailable (37) | Raised (Warning) / Cleared | Emergency / incident id |
| SosPhoneAccepted (38), SosDeliveryConfirmed (39) | Notice | Emergency / incident id |
| SosDeliveryFailed (40) | Notice (Critical) | Emergency / incident id |
| SosDeliveryUnknown (41) | Notice (Warning) | Emergency / incident id |

There are two emitters, one per task context:
- SensorProcessingTask: flight, landing and crash events share `SafetyMonitor::eventEmitter()`.
- EmergencyAlertTask: the manager's own emitter feeds a task-local queue.

Event ids are a sequence *per emitter*. The dispatcher tracks gaps and duplicates per emitter.

### 11.2 Dispatcher: events for edges, state for truth

| Input | Used for |
|---|---|
| Events | A Raised or Escalated warning restarts its sound and clears an acknowledgment. A warning raised and cleared between two snapshots is still announced (transient, 2 s). Id gaps (events lost upstream) are counted. Replayed or older ids are ignored. |
| SafetySnapshot | Which landing warnings are active now. A Cleared event lost to overflow can't leave an alert stuck. If stale (> 1 s), alerts are held 3 s, then replaced by SystemDegraded. |
| CrashStatus | CrashSuspected alert, only while no incident handles it. |
| EmergencyStatus | SOS countdown, active, communication failure, accepted. **Never from events**, so the SOS state survives any queue pressure and any sensor outage. |

### 11.3 Priorities (`AlertPriorityPolicy.hpp`, one table)

| Priority | Alert | Audio | LED | Acknowledgeable |
|---|---|---|---|---|
| 100 | SosCountdown | Tick 1/s; 2/s in the last 10 s | SOS, red | No |
| 95 | CrashSuspected | Alarm 0.7 s / 1 s | Fast, red | No |
| 92 | SosCommFailure (failed / unknown / transport down) | Low triple / 5 s | Double, red | Yes |
| 90 | SosActive | Long tone / 10 s | SOS, amber | Yes |
| 80–84 | Landing Critical | Continuous fast | Fast, red | No |
| 60–64 | Landing Warning | Triple / 3 s | Fast, amber | Yes |
| 40–44 | Landing Caution | Double / 10 s | Double, amber | Yes |
| 20–24, 25 | Landing Advisory, SosAccepted | Single or double chime, once | Slow | Yes |
| — | Nothing active | Silent | Slow, green (normal) | — |

Within a band, the kind sets the order: HazardousApproach > Closure > Descent > LowClearance >
SystemDegraded. Ties go to the most recent alert.

Consequences:
- A long-running advisory can never starve the SOS countdown.
- Any SOS state outranks every landing warning.
- The audio and the LED are chosen separately: an acknowledged alert still shows on the LED.

### 11.4 Audio and LED scheduling

`AudioAlertScheduler` turns a pattern id into timed `IAudioAlert::request({tone, priority,
durationMs, volume})` calls, one per beep, so the backend only plays a tone of known length.

- **Preemption:** cancel the current tone and start the new pattern from its first beep. A preempted
  pattern restarts from the beginning later; a half-finished beep group means nothing.
- **Gaps:** the next beep plays when due, with no burst of missed beeps. Timing is wrap-safe.
- **Backend refuses:** the state goes to `Unavailable` and requests are retried once per second, never
  every update.

`LedPatternScheduler` forwards only pattern *changes* to `IStatusLed::setPattern`, calls `tick()`, and
computes the reference on/off level `levelAt(pattern, phase)`. That function is the timing
specification for the future GPIO backend:
- slow 1 Hz;
- fast 5 Hz;
- double 2 × 100 ms per s;
- SOS ··· ——— ··· with a 150 ms unit and a 3.6 s period.

Firmware backends are `UnconfiguredAudioAlert` and `UnconfiguredStatusLed`. Both report `Unavailable`,
and nothing claims sound or light. The `IAudioAlert` tone enum and `IStatusLed` were extended
compatibly: values were appended, and `available()` has a default.

## 12. RTOS integration

- **CrashDetector** runs in **SensorProcessingTask** right after `SafetyMonitor` (option A of the
  brief). It costs a few hundred operations per frame and needs that frame's peaks and snapshot. It
  publishes a `CrashReport` (status + incident context + publish time as a heartbeat).
- **EmergencyAlertTask** is new and static:
  - priority 36: above processing (32), so a processing overload can't starve a countdown, cancel or
    retry; below acquisition (40), so sampling is never delayed;
  - 768-word stack;
  - wakes every 20 ms (`ulTaskNotifyTake` timeout) or at once on `submitCommand()` /
    `submitTransportReport()`.
- **Per wake, bounded, never blocking:**
  1. commands (≤ 8; first, so a cancel beats an expiry);
  2. transport reports (≤ 8);
  3. the crash report;
  4. `manager.tick()`;
  5. publish EmergencyStatus and PendingSosRecord;
  6. drain events (≤ 16 from processing + ≤ 8 local);
  7. dispatcher update with the latest snapshot;
  8. publish DispatchStatus.
- **The countdown advances even if sensors stop.** Nothing in the loop waits for a frame.
- **Synchronisation:**
  - one producer and one consumer per queue;
  - latest-value mailboxes for every struct crossing tasks;
  - locked bounded inboxes for commands and reports;
  - critical sections only around ≤ 188-byte copies.
- **Health:** `g_systemHealth` gained 45 fields covering:
  - crash state, evidence, counters and the last peak;
  - emergency state, incident, countdown, stage and provenance bits;
  - every counter of the brief;
  - alert audio and LED state, backlog, gaps and latency;
  - emergency task wake and work timing.

  The Health task watches the new task's stack. Large copies are kept as members, not on the 2 KiB
  health stack.

## 13. Memory and timing

All values are for the ARM build; detail is in the validation report §9.

| Item | Value |
|---|---|
| Release flash | 70 920 → 89 080 B (+18 160) |
| Static RAM (bss) | 51 112 → 59 288 B (+8 176) |
| Heap | No change (0 allocations in 30 000 updates, measured on host) |
| EmergencyAlertTask object | 5 484 B (3 072 B of it stack) |
| SensorProcessingTask object | +2 096 B |
| HealthMonitorTask | +440 B |
| `g_systemHealth` | +148 B |
| Stack, EmergencyAlert deepest chain | ~0.45 KB Release / ~0.55 KB Debug of 3 KiB |
| Stack, SensorProc `Run` | +16 B; deepest chain unchanged (~0.98 KB of 4 KiB) |
| CPU, host | Detector 26–29 ns, manager 16–19 ns, dispatcher 28–33 ns per update |
| CPU, target (estimated) | Detector 10–30 µs per frame; emergency wake < 0.1 ms. **Not measured on hardware.** |

## 14. Future Android integration (next milestone)

- A BLE transport implements `ISosTransport` on the verified EZ-Serial protocol. `submit` copies the
  64-byte frame; GATT write completion becomes `WriteCompleted`; link events become `LinkUp` /
  `LinkDown`.
- The phone app:
  - decodes v1 frames;
  - deduplicates by (device, incidentId);
  - answers with `PhoneAccepted`, `PhoneSendAttempted`, `DeliveryConfirmed` / `DeliveryFailed` and
    `CancelAccepted`, each carrying the revision;
  - may send `ManualSos`, `CancelSos`, `AcknowledgeAlert` and `ResetIncident` commands as
    `CommandSource::PhoneApp`.
- The app must not report "confirmed" for an API success alone.

## 15. Limitations (also in the validation report)

- The thresholds, countdowns and retry timings come from synthetic simulations. There is no real
  incident data.
- 100 Hz polling and 16 g / 2000 dps saturation limit impact observation.
- **A hard landing after which the pilot lies still is indistinguishable from a crash, and so is a
  very hard (≥ 8 g) landing in flight.** Both are suspected by design and rely on the countdown for
  cancellation. Validation report §4.3.
- Being dragged on the ground by the wing at ≥ 6 m/s within ~5 s of an impact, with LiDAR
  unavailable, could look like resumed flight.
- Abnormal orientation relies on Madgwick after a violent impact, which may need seconds to re-level.
  The IMU mounting is still a placeholder.
- Hardware is not configured:
  - no transport (the request stays pending, honestly);
  - no audio or LED;
  - no persistence (no continuation across a reset);
  - no battery value in the request.
- The emergency task's emitter and the processing emitter have separate event-id sequences (per-source
  gap tracking).

## 16. Rejected alternatives

| Alternative | Why rejected |
|---|---|
| Single \|a\| threshold → SOS | Spirals, hard landings and drops exceed it. Brief §6. |
| Crash detection on UKF / Madgwick output | Smoothing and gating remove the impulse. |
| Voted specific force for the impulse | The voter drops saturated channels, so the hardest impacts would vanish. |
| Max over IMUs | One faulty IMU would create impacts. The second-largest is robust to one fault. |
| Probabilistic score | Unvalidated weights would look more certain than they are. Counted, named indicators are explainable and testable. |
| Countdown in SensorProcessingTask | Stops when frames stop. |
| Emergency task below processing | A processing overload could starve the countdown. |
| SOS state carried by events | The queue drops advisories under pressure. State comes from the manager's status instead. |
| Auto-giving-up when no link | The request would be silently lost. It stays pending with an explicit flag. |
| Treating BLE write completion as delivery | It proves nothing about the phone or a contact. |
| Persisting every tick | Flash wear and blocking. Writes happen on transitions only, through an asynchronous backend later. |
