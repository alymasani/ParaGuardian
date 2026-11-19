# Crash Detection, Emergency Manager and Alert Dispatch: Validation Report (Milestone 6)

**Everything below is from host simulation and host / ARM builds. Nothing has run on hardware.**
- Synthetic results are **not** real-world sensitivity, specificity or false-alarm rates.
- The transport, phone, audio and LED are recording mocks. They show the firmware's bookkeeping, not
  that a message, sound or light reached anyone.
- All thresholds are provisional.

## 1. Test counts and environment

| | Value |
|---|---|
| Baseline (start of Milestone 6, before any edit) | **393 / 393** passed (`sh Tests/run_host_tests.sh`) |
| New tests | **70**: `test_crash_detector.cpp` 19, `test_emergency_manager.cpp` 25, `test_alert_dispatch.cpp` 13, `test_crash_emergency_integration.cpp` 13 |
| Final | **463 / 463** passed: plain (`SANITIZE=0`), ASan + UBSan (`SANITIZE=1`) and ThreadSanitizer (`SANITIZE=thread`), section 11 |
| Old tests changed | None. The 393 earlier tests are unchanged and pass. |
| Host | Apple clang, `-std=c++17 -O1 -Wall -Wextra -Wpedantic -Wshadow -Werror` |

Scenario categories 1–70 of the brief are covered by fewer, larger tests. Each table row below is an
asserted case unless it is marked "metric".

## 2. Simulation method (independent truth)

`Tests/sim/CrashSimulator.hpp` builds on the Milestone 5 simulator (`fsim`), which already provides
truth kinematics, terrain, LiDAR ray tracing, navigation / GNSS / attitude inputs and the **real**
`SafetyMonitor` (phase, clearance, landing warnings). It adds:
- **Ground contact detected from the truth** (the device reaches the terrain while descending), not
  scripted. Impact pulses and post-contact behaviour (tilt, stillness, walking) are placed relative
  to the detected contact.
- **Three IMU channels sampled at their own read instants** inside each 10 ms cycle (3, 2, 1 ms before
  the frame). Short pulses can be missed or under-read, as with the real 100 Hz polling.
- **Signal model:**
  - sustained load factor;
  - motion noise;
  - half-sine impact pulses (g, duration) and rotation pulses (dps);
  - clipping at 16 g / 2000 dps with saturation flags;
  - per-channel faults (offsets, spikes, absence);
  - total IMU dropout.
- The voted force is the median of the unsaturated channels, matching the redundancy voter's role.

Scenarios are *classes of motion*, not recordings. The expected outcome of every class was written down
before it was run (`expectSuspicion` / test assertions).

## 3. Crash detector: synthetic incidents and normal activity

### 3.1 Normal flight and handling: no suspicion expected

| # | Scenario | Candidates | Suspected | Decision |
|---|---|---|---|---|
| 1 | Steady glide, 150 s | 0 | **0** | — |
| 2 | Thermal climb (30° bank, 1.15 g, 0.35 rad/s) | 0 | **0** | — |
| 3 | Turbulence: 40 pulses of 1.6–2.9 g, 100–220 dps | 0 | **0** | — |
| 4 | Coordinated turn (45°, 1.41 g) | 0 | **0** | — |
| 5 | Spiral dive (3.6 g sustained, 150 dps, 70° bank, −14 m/s) + wingovers (0.7–2.5 g) + collapse (3 g, 250 dps) | 0 | **0** | — |
| 6 | Normal landing (1.8 g touchdown), pilot walks | 0 | **0** | — |
| 7 | Hard but controlled landing (4.5 g, −2.5 m/s), pilot walks | 1 | **0** | InsufficientEvidence |
| 8 | Device dropped on the ground (0.45 s free fall, 12 g, lands on its side) | 1 | **0** | DropSignature |
| 9 | Picked up and shaken (24 pulses 2–6 g, 300–600 dps) | 11 | **0** | NoFlightContext |
| 10 | Failed takeoff: lifts ~3 m, falls back (5 g), pilot gets up | 1 | **0** | InsufficientEvidence |
| 21 | IMU0 alone spikes to 12 g in flight | 0 (3 uncorroborated peaks counted) | **0** | — |

### 3.2 Incidents: suspicion expected (latency = suspicion − true ground contact)

| # | Scenario | Suspected | Latency | Decision | Notes |
|---|---|---|---|---|---|
| 11 | Collapse, −9 m/s, 10 g + 600 dps, lying at 80° | yes | **2.01 s** | StrongImpactWithContext | pre = airborne, fast descent, ground closure |
| 12 | High-rotation impact (6 g, 900 dps), −3 m/s | yes | 2.02 s | StrongImpactWithContext | |
| 13 | Low-speed impact (4.5 g) with fast sink (−5 m/s), lying 70° | yes | 2.53 s | ImpactWithPostSupport | moderate impulse |
| 14 | 7 g impact then inactivity | yes | 4.52 s | ImpactWithPostSupport | waits for 4 s of inactivity |
| 15 | Strong impact (10 g), pilot keeps moving vigorously | yes | 10.01 s | StrongImpactFlightEnded | PilotActivity reported; decided at the window end |
| 16 | #11 without GNSS | yes | 2.01 s | StrongImpactWithContext | `GnssMissing` |
| 17 | #11 without LiDAR | yes | 2.01 s | StrongImpactWithContext | `LidarMissing` |
| 18 | #11 with barometers / navigation failed | yes | 2.01 s | StrongImpactWithContext | `NavigationMissing` |
| 19 | 25 g impact, all IMUs saturate at 16 g | yes | 2.01 s | StrongImpactWithContext | saturated: peak is a lower bound |
| 20 | One IMU failed for the whole flight | yes | 2.01 s | StrongImpactWithContext | 2-IMU corroboration |
| 21b | IMU0 reads +3 g off, real crash | yes | 2.01 s | StrongImpactWithContext | faulty IMU outvoted |
| 22 | All IMU data lost 0.1 s after a strong impact (slow sink) | yes | 10.01 s | StrongImpactDataLoss | explicit missing-data policy |
| 23 | Device started while airborne (Unknown → Flying), later crash | yes | 2.01 s | StrongImpactWithContext | |
| 24 | Tumbling: three impacts 0.8 s apart (5 g each), lying 65° | yes | 2.02 s | StrongImpactWithContext | `RepeatedImpacts`, impacts = 3 |
| 25 | #11 with the MCU clock wrapping during the flight | yes | 2.01 s | same as without the wrap | identical evidence |

**Synthetic totals (tuning classes):**
- 15 / 15 incident classes suspected.
- 0 suspicions in 11 normal or handling classes.
- Latency 2.0–10.0 s (median ≈ 2 s).

These are synthetic; they say nothing about real-world rates.

### 3.3 Unit checks of the detector (asserted)

- A 12 g spike on the ground (never airborne) is never suspected.
- A strong impact in a fast descent is suspected exactly at `minObservation` (22.0–22.1 s for an impact
  at 20.0 s).
- NaN, ±∞ and negative peaks never become valid numbers.
- Duplicate frames: the same peak timestamp is counted stale. Older frames are ignored entirely.
- One faulty IMU can't create (14 g alone) or hide (dead IMU during a 10 g impact) an impulse.
- A single IMU is capped at Moderate.
- A sustained spiral ramp to 4.5 g creates no candidate.
- No IMU for 1 s → Unavailable; data back → Monitoring.
- After a suspicion: 60 s hold, then 30 s cooldown. Impacts are counted, no new suspicion, then re-armed.
- Flight resumes → `CrashSuspected/Cleared`, `suspicionsCleared` = 1.
- The same sequence across the uint32 wrap gives identical evidence and decision.
- A 10 s frame gap sets `HistoryGap`.

### 3.4 Fixes found by testing

**1. Spiral rotation false alarm.** In the first version, a rotation rate ≥ 400 dps counted as an
impulse without any baseline. In scenario 5 the spiral's ~150 dps with jitter produced a strong
"impact" (≥ 4 g and ≥ 400 dps) and a **false suspicion**.
- Fix: a rate baseline (EMA) and `rateImpulseMin` = 250 dps, mirroring the force baseline.
- The simulator's rate jitter was also reduced from ±50 % to ±10 % (its own truth model) as more
  realistic.

**2. Turbulence in calm air (design review, not a test failure).** `resumedFlightMs` was cut from 5 s
to 3 s in the same change, so a strong turbulence hit followed by calm air is recognised as flight
(Rejected `FlightResumed`) before 4 s of calm could look like inactivity. This was decided before the
held-out set was written; held-out H9 tests exactly this situation.

Test-side corrections (not detector changes):
- the cooldown test waited too briefly;
- scenarios 15 and 22 used a sink rate that triggered GroundClosure, so their intended decision path
  was not exercised;
- scenario 15's "moving" pilot was too gentle to register as activity. It was made vigorous (4 m/s²).

## 4. Threshold sensitivity, held-out set and ambiguity

### 4.1 Sweeps (tuning set: 5 incidents T11–T15, 4 normals: glide, landing, hard landing, turbulence)

| Configuration | Detected | False alarms | Mean latency |
|---|---|---|---|
| **Default** (4 g, 400 dps, inactivity 4 s, observation 10 s) | 5 / 5 | 0 / 4 | 4.22 s |
| impactAccel 3.0 g | 5 / 5 | 0 / 4 | 4.21 s |
| impactAccel 5.0 g | **4 / 5** | 0 / 4 | 4.64 s |
| impactAccel 6.0 g | **4 / 5** | 0 / 4 | 4.64 s |
| impactRate 300 dps | 5 / 5 | 0 / 4 | 4.22 s |
| impactRate 600 dps | 5 / 5 | 0 / 4 | 4.22 s |
| candidateWindow 200 ms | 5 / 5 | 0 / 4 | 4.10 s |
| inactivity 2 s | 5 / 5 | 0 / 4 | 3.82 s |
| inactivity 8 s | 5 / 5 | 0 / 4 | 5.02 s |
| observation 6 s | 5 / 5 | 0 / 4 | 3.42 s |
| observation 14 s | 5 / 5 | 0 / 4 | 5.02 s |
| quietForce 0.4 m/s² | 5 / 5 | 0 / 4 | 4.22 s |

Reading:
- Raising the impact threshold to 5 g or more **misses the 4.5 g low-speed incident** (T13).
- Lowering it to 3 g changes nothing on this set. Lower thresholds will matter with real turbulence,
  which this synthetic set does not capture.
- Inactivity and observation time trade latency against evidence. The window end bounds the
  strong-impact-with-movement case (T15 = observation time).
- The countdown duration only shifts the time from contact to request: 32.0 s, 62.0 s and 122.0 s for
  30, 60 and 120 s countdowns.
- The cooldown and re-arm time (60 s hold + 30 s cooldown) bound how soon a *new* suspicion can open.
  It was not swept against data, because none exists.

### 4.2 Held-out scenarios (parameters frozen before running; expected outcome stated in the test)

| Case | Expected | Got | Latency / decision |
|---|---|---|---|
| H1 sink −7 m/s, 8 g + 450 dps, lying at 50° | suspect | suspect | 4.51 s, ImpactWithPostSupport |
| H2 hard landing −3 m/s, 5.5 g, walks away | none | none | InsufficientEvidence |
| H3 strong turbulence (to 3.2 g), different seed | none | none | — |
| H5 dropped 20 s after landing (0.4 s free fall, 9 g) | none | none | DropSignature |
| H6 sink −6 m/s, 12 g, unconscious, upright | suspect | suspect | 2.01 s |
| H7 crash with GNSS **and** LiDAR missing | suspect | suspect | 2.01 s |
| H8 slow tree-like contact (−2 m/s, 3 g), moving | none | none | — |
| H9 in-flight collapse 4.3 g + 450 dps, flight continues | none | none | FlightResumed (the 3 s resumption value of §3.4 predates this case) |

Result: 8 / 8 as expected. The held-out set is small and synthetic.

### 4.3 Cases the algorithm cannot separate (asserted, by design)

| Case | Outcome | Why |
|---|---|---|
| Hard landing (4.5 g), pilot then lies still on the back (85°) | **suspected** (ImpactWithPostSupport) | Same observations as a crash with an unconscious pilot. The countdown asks the pilot. |
| Very hard landing (9 g), pilot walks away | **suspected at the window end** (StrongImpactFlightEnded) | A strong in-flight impulse after which flight did not resume. The pilot cancels. |

The sensors don't measure injury. Reporting these as ambiguous, and letting the countdown resolve them,
is the intended behaviour, not a solved classification problem.

## 5. Emergency manager

| # | Scenario | Result (asserted) |
|---|---|---|
| 26 | Suspicion starts the countdown | Countdown, incident 1, episode linked, `EmergencyCountdown/Raised`, remaining 60 000 ms |
| 27 | Suspicion cleared before escalation | Cancelled (SuspicionCleared); nothing sent; re-armed |
| 28 | Pilot cancels | Cancelled, `cancelNotice = None`, events Countdown/Cleared + Cancelled; Idle after 10 s; nothing sent |
| 29 | Countdown accuracy and the boundary | Request created **60 001 ms** after the start (1 ms ticks: strictly after the deadline). Cancel issued **at** the deadline: in time. Cancel 1 ms later: the request exists but never left, so it is withdrawn locally. |
| 30 | Manual SOS without crash evidence | Trigger Manual; handed to the transport 5 020 ms after the command (5 s countdown + 20 ms tick); `Impact` bit clear |
| 31 | Repeated manual SOS | Same sequence → Duplicate. New press during the countdown → escalate now. Presses while sending → NoEffect. One incident, one request. |
| 32 | Invalid or malformed commands | Wrong id → WrongIncident; zeroed / id 0 cancel / bit-flipped / bad type → Malformed. No state change. A malformed manual SOS creates nothing. |
| 33 | Countdown expires | Request Created, `EmergencyRequest` notice |
| 34 | Transport unavailable | SosPending + `TransportUnavailable` raised. **10 min without a link: 0 attempts, never failed.** The pending record stays active. Link up → sent immediately, flag cleared. |
| 35–36 | Disconnect, reconnect, retry | WriteCompleted alone = TransferredToPhone (not accepted). LinkDown → SosPending. LinkUp → re-sent, same incident id. Accepted on attempt 2. |
| 37 | Duplicate acknowledgment | 2 duplicates counted; no new event, state or incident |
| 38 | Delayed ack, retry timing, exhaustion | Attempt spacing **12 000 ms then 14 000 ms** (10 s timeout + 2 s, 4 s backoff). A late acceptance of revision 1 is valid. Never acknowledged: **8 attempts → DeliveryFailed / RetriesExhausted after 262.1 s**. Reconnect → cycle 2, same incident. |
| 39 | Wrong incident / future revision / unknown kind | 2 wrong-incident and 2 invalid reports counted; no state change |
| 40 | Accepted, no outcome | **DeliveryUnknown** after 120 s (never success); a later confirmation → Delivered |
| 41 | Explicit delivery failure | DeliveryFailed (PhoneReportedFailure). Manual SOS → re-sent, same incident, revision 2. |
| 42 | Delivery provenance | Transport acceptance = QueuedForTransport only. Full chain → all six stage bits, no `ProvenanceIncomplete`. A failure after a confirmation is ignored. Confirmation without acceptance → Delivered + `ProvenanceIncomplete`, AcceptedByPhone bit **clear**. |
| 43 | Sensor outage during the countdown | No crash reports after the start; ticks alone create the request |
| 44 | Scheduler gap | No tick for 75 s: expiry evaluated **15 000 ms late**, flagged `CountdownOverrun`. A cancel issued at 30 s and processed after the gap is honoured. |
| 45 | Reboot with mock persistence | Request in flight at reset → SosPending, same incident id, revision 2, `Restored` bit, re-sent. Ids continue after reset. Countdown at reset → **full** countdown restarts. Corrupt record → ignored. No store → `PersistenceUnavailable | IncidentIdVolatile`. |
| 46 | Re-arm and new incident | A second suspicion during the countdown → `additionalImpacts` = 1, still one incident. Reset while sending → NotAllowed; wrong id → WrongIncident; after delivery → Idle. A duplicate ack after the reset creates nothing. A new suspicion → incident 2. |
| — | Cancel after the request left | `cancelledAfterSend`; a Cancel-kind request (same incident) is sent; CancelAccepted → `CancelNotice::Accepted`; re-arm |
| — | Detector silence | A strong in-flight impact under observation, then no new reports for 5 s → incident (DetectorSilence), once per episode |
| — | Invalid configuration | Unavailable; commands NotAllowed |

SOS request checks:
- 64-byte round trip.
- **All 512 single-bit corruptions rejected.**
- Unknown version rejected even with a valid CRC.
- Too-small buffer → 0.
- GNSS MSL (12345 dm) and barometric relative altitude (123 dm) kept apart.
- Fix age stated.
- No fix → Position bit clear, `fixAgeMs` = 0xFFFFFFFF.
- A 99 s-old fix → `PositionStale`.
- Navigation older than 5 s is not sent.
- Legacy payload uses MSL only, battery 0 (unknown).
- The persistence record round-trips and rejects corruption.
- The command inbox drops the newest when full (counted) and stays FIFO.

## 6. Alert dispatcher and outputs

| # | Scenario | Result (asserted) |
|---|---|---|
| 47 | Advisory and caution patterns | LowClearance advisory: **one** chime in 30 s. Descent caution: 6 beeps in 30 s (2 per 10 s); LED double / amber. |
| 48 | Critical terrain preempts an advisory | Same update: CriticalFast tone requested, preemption counted, LED fast red; advisory still listed as active |
| 49 | SOS countdown preempts a landing warning | Countdown tick, LED SOS red; CountdownFinal (2/s) in the last 10 s; acknowledgment refused; after cancel the landing warning resumes |
| 50 | Repeated / replayed events | Duplicates ignored (2 counted); no restart, no cancel |
| 51–52 | Clear and overlap | Warning outranks Caution; when it clears, the caution takes over; all clear → silent, LED normal (slow green) |
| 53, 54, 59 | Queue overflow | 60 advisories + 1 critical into a 16-slot queue with no consumer: **critical kept**, `droppedCritical` = 0, **44 lost ids counted** as gaps. An SOS being sent is still the top alert (from status). |
| 55 | Audio unavailable | State Unavailable, 4–6 failures in 5 s (retried ~1/s, not every update); LED unaffected. Null and Unconfigured backends report Unavailable. |
| 56 | LED unavailable | State Unavailable, no `setPattern` calls; audio unaffected |
| 57 | No sensor frames | Countdown beeps continue. Landing alert held 3 s, then released and replaced by SystemDegraded. |
| 58 | Wraparound | 1 beep per second across 0xFFFFFFFF; after a 5 s gap one beep, no burst |
| — | Acknowledge | Warning silenced (LED still shows it); escalation to Critical sounds again |
| — | Transient | Raised and cleared between snapshots: still announced once, then released |
| — | LED timing | `levelAt` reference values for slow, fast, double and SOS |

Dispatch latency: an event is consumed in the next emergency wake (≤ 20 ms on target, by design).
`lastLatencyMs` and `maxLatencyMs` are reported in `g_systemHealth`. The maximum processing-queue
occupancy in the end-to-end runs was 3 of 16.

## 7. End-to-end (simulated flight → detector → manager → dispatcher → mocks)

| # | Scenario | Result |
|---|---|---|
| 61 | Flight + safe landing | 0 incidents, 0 requests; 1 advisory chime (low clearance on approach) |
| 62 | Turbulent flight (40 pulses to 3.2 g) | 0 incidents, 0 requests |
| 63 | Suspected crash, cancelled by the pilot 20 s into the countdown | contact 44.63 s, suspicion 46.64 s, cancelled 66.66 s, 0 requests, 0 sent; ≥ 15 countdown beeps; re-armed |
| 64 | Suspected crash → countdown → SOS → phone → delivery | contact 44.63 s; suspected **+2.01 s**; request **+62.03 s** (60 s countdown); submitted +62.03 s; phone accepted +62.83 s; delivery *reported* +70.03 s (mock delays 0.8 / 1.5 / 8 s). Position and impact in the request. |
| 65 | Manual SOS in ordinary flight | command 60.0 s → request 65.02 s → delivered 73.02 s; no crash suspicion; `Impact` bit clear |
| 66 | Emergency with GNSS lost | Delivered; request **without position** (bit clear, age 0xFFFFFFFF) |
| 67 | Emergency with LiDAR lost | Delivered; position and relative altitude present |
| 68 | Emergency with navigation failed | Delivered; relative altitude **not** sent (stale) |
| 69 | Transport down when the request is created; back 90 s later | SosCommFailure alert shown meanwhile; submitted 90 s later; delivered; **1 attempt** (waiting did not consume attempts) |
| 70 | Ten consecutive rounds (crash, normal, crash + cancel, manual, …) on one manager / dispatcher | 7 incidents (7 expected), 5 delivered, 2 cancelled, 0 wrong-incident reports, 0 event gaps; always back to Idle or Cancelled |

## 8. Fault injection summary

| Fault | Behaviour |
|---|---|
| GNSS, LiDAR or barometers lost | Detection continues on the remaining evidence and the missing source is flagged. The SOS states which fields are valid. |
| One IMU failed or offset / spiking | Corroboration outvotes it; uncorroborated peaks counted |
| IMU saturation | Flagged; strong class; lower-bound value |
| All IMU data lost after an impact | Explicit data-loss policy at the window end |
| Processing task stalls after an impact | DetectorSilence incident after 5 s |
| NaN / Inf / negative inputs | Ignored, never valid zeros |
| Duplicate or out-of-order frames, events, commands, acks | Ignored and counted |
| uint32 clock wrap | Detector, manager and audio verified across the wrap |
| Large gaps (75 s) | Countdown expires late and flagged; cancel honoured by its issue time |
| Event queue overflow | Critical kept; gaps counted; SOS state unaffected |
| Transport down / phone disconnects / no ack / wrong incident / failure | Sections 5 and 7 |
| MCU reset | Mock persistence restores; firmware has none (stated) |
| Audio or LED backend missing | Unavailable state; the other output unaffected |

## 9. Resources (ARM, STM32CubeIDE 2.2.0, GNU Tools for STM32 14.3)

| | Before (end of Milestone 5) | After | Change |
|---|---|---|---|
| Release text | 70 920 | **89 080** | +18 160 |
| Release bss | 51 112 | **59 288** | +8 176 |
| Debug text | 122 804 | **156 636** | +33 832 |
| Debug bss | 51 120 | **59 288** | +8 168 |
| data | 104 | 104 | 0 |

Static RAM breakdown (Release, `nm -S`):

| Object | Size |
|---|---|
| EmergencyAlertTask object | 5 484 B (stack 3 072, manager 480, dispatcher 456, inboxes 148 + 116, local queue 360, mailboxes, copies) |
| SensorProcessingTask | 21 160 → 23 256 B (+2 096: CrashDetector 1 560, CrashReport 164, IncidentContext 52, mailbox, lock) |
| HealthMonitorTask | 3 300 → 3 740 B (+440: three copies kept off its stack) |
| `g_systemHealth` | 756 → 904 B (+148: 45 fields) |

ARM struct sizes:

| Struct | Bytes | Struct | Bytes |
|---|---|---|---|
| CrashDetector | 1 560 | EmergencyStatus | 152 |
| CrashInput | 124 | SosRequest | 64 |
| CrashStatus | 108 | PendingSosRecord | 76 |
| CrashReport | 164 | SafetyEventDispatcher | 456 |
| IncidentContext | 52 | DispatchStatus | 76 |
| EmergencyManager | 480 | EmergencyCommand | 16 |
| TransportReport | 12 | AudioAlertScheduler | 52 |
| LedPatternScheduler | 24 | | |

Stack (`-fstack-usage`):

| Task | Measure | Value |
|---|---|---|
| EmergencyAlertTask | `Run` | 184 B Release / 232 B Debug |
| EmergencyAlertTask | Deepest chain (command → expiry → request → event → queue) | ≈ 0.45 KB Release / 0.55 KB Debug of 3 KiB |
| SensorProcessingTask | `Run` | 296 → 312 B |
| SensorProcessingTask | Crash chain | ≈ 0.6 KB |
| SensorProcessingTask | Deepest chain (navigation, unchanged) | ≈ 0.98 KB of 4 KiB |

Heap: **0 allocations** in 30 000 combined detector + manager + dispatcher updates (global
`operator new` counted; scenario 60). No steady-state heap was introduced.

CPU:
- Host (Apple M4, `-O1`) per update: detector 26–29 ns, manager tick 16–19 ns, dispatcher 28–33 ns.
- Target: **estimated** 10–30 µs per detector update and < 0.1 ms per emergency wake at 64 MHz, using
  the Milestone 3 host-to-target ratio. **Not measured on hardware.**
- The emergency task's utilisation estimate is < 0.5 % at 50 Hz, and its preemption of SensorProc is
  < 0.1 ms per 20 ms.

## 10. Builds

| Build | Result |
|---|---|
| CubeIDE Debug (headless, clean scratch copy) | **0 errors, 0 warnings**; text 156 636, data 104, bss 59 288 |
| CubeIDE Release | **0 errors, 0 warnings**; text 89 080, data 104, bss 59 288 |
| Strict ARM compile of every new or changed `.cpp` (`-Wall -Wextra -Wshadow -Wconversion -Wsign-conversion -Wdouble-promotion -Wcast-align -Wnull-dereference`) | **0 warnings** |

Baseline note: the CubeIDE builds were **not re-run at the start of this milestone**. The "before"
figures are the Milestone 5 final builds of the same working tree. No file changed between that build
and the first Milestone 6 edit; `git status` was identical (46 entries). The build method was the same
headless CubeIDE build on a clean scratch copy.

## 11. Sanitizers

| Run | Result |
|---|---|
| ASan + UBSan (`SANITIZE=1`) | 463 / 463, **0** sanitizer reports |
| ThreadSanitizer (`SANITIZE=thread`) | 463 / 463, **0** sanitizer reports |

## 12. Acceptance criteria

| # | Criterion | Status |
|---|---|---|
| 1 | Portable crash detector | Met (`Algorithms/Crash`, no HAL / RTOS / heap) |
| 2 | Multiple evidence sources | Met (impulse, rotation, saturation, free fall, phase, sink, LiDAR closure, GNSS, inactivity, orientation, repeated impacts) |
| 3 | IMU evidence before UKF smoothing | Met (per-IMU peaks in the frame; real-pipeline test) |
| 4 | Suspicion does not imply injury | Met (terminology, decision codes, docs) |
| 5 | Bounded temporal context | Met (32 × 250 ms history, time windows, wrap-safe) |
| 6 | Independent emergency state machine | Met |
| 7 | Automatic suspected-crash escalation | Met (26, 33, 64) |
| 8 | Manual SOS independent of detection | Met (30, 65) |
| 9 | Non-blocking countdown and cancellation | Met (28, 29, 63) |
| 10 | Countdown advances without sensors | Met (43, 57; task design) |
| 11 | Incident ids and correlation | Met (32, 39, 46) |
| 12 | Bounded, idempotent retries | Met (36, 38, 41) |
| 13 | Transport ≠ delivery | Met (35, 40, 42) |
| 14 | SOS survives event-queue overflow | Met (34, 53/59; `PendingSosRecord`) |
| 15 | Landing queue has a consumer | Met (EmergencyAlertTask dispatcher) |
| 16 | Priorities and preemption | Met (48, 49, 52) |
| 17 | Audio / LED scheduling with mocks | Met (47–58) |
| 18 | No unverified audio / BLE claim | Met (Unconfigured backends; docs) |
| 19 | Acquisition not blocked | Met by design (priorities 40 > 36 > 32, no blocking calls); **not measured on hardware** |
| 20 | No steady-state heap | Met (0 allocations measured) |
| 21 | 393 existing tests pass | Met |
| 22 | New tests for crash, emergency, alert | Met (70) |
| 23 | End-to-end simulated emergencies | Met (61–70) |
| 24 | CubeIDE Debug and Release build | Met (0 / 0) |
| 25 | SoarOS unchanged | Met (426c1ef, clean) |
| 26 | Git uncommitted and unstaged | Met (CRASH_EMERGENCY_GIT_SUMMARY.md) |
| 27 | Documentation accurate | Met to the best of review: numbers are copied from the runs above |

Not met or partial (stated plainly):
- **Persistence is interface + mock only.** Firmware SOS continuation across a reset is not possible.
- No hardware timing, stack or audio / LED / BLE verification.
- Baseline builds were not re-run at the start (section 10).

## 13. Unverified assumptions

1. Impact pulse shapes and magnitudes (half-sine, 25–80 ms, 4–25 g) and post-impact behaviour are
   assumptions, not measurements.
2. Real turbulence, hard landings, harness handling and drops may exceed the synthetic magnitudes.
3. The 100 Hz polling and the LSM6DSRX filter configuration may under-read real impacts.
4. The Madgwick tilt after a violent impact converges within the 2 s abnormal-orientation window.
5. The flight-phase detector (Milestone 5) is correct enough to provide "airborne" context.
6. Countdown durations, retry and backoff timings, and delivery timeouts suit the future phone app and
   the rescue process.
7. The future BLE transport and phone app will report stages faithfully.
8. Target CPU times are estimates (host × ratio).
