# Flight State, Ground Clearance and Landing Safety: Validation Report (Milestone 5)

**Scope:** host simulation and independent geometric checks, sanitizers, strict compiler checks, and native
STM32CubeIDE firmware builds. **No hardware, no LiDAR mounting, no sensor or flight data.** Nothing here
qualifies the system for flight or safety use. Synthetic scenarios are classes of motion, **not**
recordings of real paraglider flights. Every number comes from the final test run (`[flight]`,
`[safety]`, `[landing]`, `[integ]`, `[sweep]` lines) or the final CubeIDE build, unless it is marked
**estimate**.

---

## 1. Summary

| Item | Result |
|---|---|
| Tests before the milestone | **338 / 338 passed** |
| New tests | **55**: ground clearance 13, flight state 14, landing safety + queue 16, integration / faults / sweeps 12 |
| Final tests | **393 / 393 passed**; also 393 / 393 under ASan+UBSan and under TSan, 0 sanitizer reports |
| LiDAR projection vs independent ray–plane truth (flat) | max error 2.3·10⁻⁵ m (8·10⁻⁷ relative) over 849 attitudes; all 1 150 out-of-limit geometries refused (no manufactured clearance) |
| Flight-phase scenarios (A1–A15 + extras) | all expected phase sequences; **0 false Flying, 0 false Landed** in the stationary / carried / dropped / soaring / outage cases |
| Normal flights (20, 10 held-out seeds) | **0 Caution-or-higher warnings**; 10 advisories (one `LowGroundClearance` per landing approach) |
| Synthetic hazards (C33, C34, C35/38, D52) | all detected; e.g. closure Critical 0.04 s after truth TTC < 2 s and 1.0 s before the ground is 6 m away |
| Event queue | 0 critical events lost under 10× slow consumption |
| CubeIDE Debug / Release | 0 errors / 0 warnings each; Release text +11 584 B, bss +3 088 B |
| SoarOS | unchanged (`426c1ef`, clean) |

---

## 2. Baseline (recorded before any change)

| Check | Result |
|---|---|
| Git | branch `Pre-processing`, HEAD `c26ff8c`; 33 uncommitted entries from Milestones 3–4 (12 modified, 21 untracked); **nothing staged**; no stash |
| SoarOS | `426c1efe… (heads/master)`, clean |
| Host tests | 338 / 338 |
| CubeIDE Debug | 0 / 0; text 102 308, data 104, bss 48 032 |
| CubeIDE Release | 0 / 0; text 59 336, data 104, bss 48 024 |

The Milestone 4 claims were confirmed against the repository before editing.

---

## 3. Simulation methodology (independence)

`Tests/sim/FlightSafetySimulator.hpp`:

* **Truth kinematics:** vertical speed, ground speed, heading, roll and pitch are functions of time.
  Position is integrated in double at 1 ms. The device cannot go below terrain + harness height.
  "Airborne" truth = more than 0.3 m above the ground-contact height.
* **Terrain:** an arbitrary height function (flat, slopes, hill launch, cliffs, vegetation patches).
* **LiDAR truth:** the TRUE beam is rotated by the simulator's **own Euler matrices** (not the
  firmware quaternion code). It is marched through the terrain in 2 cm steps, then refined by
  bisection in double. No hit within 40 m = out-of-range rejection, as from the driver. Range noise
  follows the TF02-Pro manual. Rejected samples do not replace the accepted one, as in preprocessing.
* **Geometry tests:** additionally use a closed-form **ray–plane intersection**.
* **Other inputs:** navigation, attitude, GNSS and IMU motion are generated from truth with noise,
  outages and quality states. Navigation vz noise is 0.12 m/s (σ 0.15) and altitude noise 0.3 m. The
  layout is exactly what `safetyInputFrom()` produces.
* **Real data path:** D55 runs the REAL preprocessing, Madgwick and UKF on simulated sensor messages,
  with LiDAR and GNSS messages injected through the pipeline.
* **Expectations come from truth:** true lift-off and touchdown, true clearance and TTC, true sink. They
  are not taken from the thresholds.
* **Seeds:** tuning used seeds 1–3 (§9). All scenario results below use held-out seeds 101–110, with the
  defaults frozen.

---

## 4. Ground-clearance accuracy

| Test | Result |
|---|---|
| Flat terrain, 2 000 random attitudes (roll / pitch ±55°, any yaw, h 2–32 m) | 849 valid: max \|error\| **2.26·10⁻⁵ m**; 1 150 refused (tilt > 40° or no hit): none produced a number |
| 30° bank / 30° + 10° pitch / 35° pitch | 10.000 m (truth 10.0) each; slant 11.55 / 11.73 / 12.21 m reported separately; `HighTilt` set |
| 60° bank, 45° pitch | refused (`BeamGeometry`) |
| Beam horizontal / upward / upside down | refused; NaN |
| Sloped terrain (15 m above, vertical beam) | 0.00 m error vs beneath |
| Sloped terrain with 20° bank (vs clearance **beneath**) | 10°: +1.03 / −0.90 m; 20°: +2.29 / −1.75 m; 30°: +3.99 / −2.60 m. Exactly equal to the illuminated-point truth (< 1 mm), so the bias is the flat-terrain assumption, documented |
| Incorrect mounting (true beam 10° off the configured axis) | up to **0.85 m** error at 10 m; outputs flagged `MountingUnverified`. The correct configured mounting is exact |
| Cliff −12 m / vegetation +4 m (32 m clearance, 10 m/s) | new value reported at once: 32.20 m (truth 32), 28.00 m (truth 28); 2 step flags; largest closure rate reported 0.95 m/s (truth 0); error vs illuminated truth RMS < 0.4 m, max < 1.4 m (1 % noise at 32 m) |
| Out of range | `BeyondRange`, lower bound 40.0 m (level), 34.64 m (30° bank) |
| Weak / saturated, invalid, never present | `WeakSignal` / `InvalidMeasurement` / `NoLidar`, NaN |
| Republished sample ×5 | `NotNew` ×5, 1 processed sample; stale after 60 ms |
| Attitude invalid / after gap / 70 ms skew | refused (`AttitudeInvalid`, `AttitudeStale`) |
| uint32 wrap-around | bit-identical clearance and rate to a run at normal times |

### Closure rate and TTC

| Test | Result |
|---|---|
| Rising terrain 1:5 at 9 m/s, UKF vz = 0 (true closure 1.8 m/s) | estimate **1.99 m/s**, clearance 22.3 m, TTC 11.2 s (= c/rate); over the run: error RMS 0.381, p95 0.775, max 1.645 m/s (n 781). Mean reported σ 0.328 m/s, so the reported uncertainty matches the noise of 1 % range at 20–30 m |
| Steady 2 m/s descent, 100 samples | 2.000 m/s; TTC = c/2 |
| One sample / climbing away / one 6 m spike | no rate; no TTC; spike flagged, rate invalid (no fake closure) |

**Availability:** with 50° wingovers at 25 m (C40), 36.2 % of frames were refused for beam geometry,
and clearance was available in 59.1 % of frames. This is the cost of refusing tilted beams.

---

## 5. Flight-state results

Phase codes: 1 Ground, 2 TakeoffCandidate, 3 Flying, 4 Approach, 5 LandingCandidate, 6 Landed.

| Scenario | Phase sequence (time s : phase) | Truth | Delay / outcome |
|---|---|---|---|
| A1 stationary 120 s | 5.0:1 | ground | Ground after 5.0 s; no other transition |
| A2 picked up 1 m | 5.0:1 | ground | no candidate, no Flying |
| A3 carried uphill +26 m, 1.3 m/s | 5.0:1 | ground | no candidate, no Flying |
| A4 hill launch (LiDAR + GNSS + nav) | 5.0:1 14.0:2 17.6:3 18.6:4 33.8:3 | lift-off 13.2 | **Flying 4.33 s after lift-off** (Approach while < 30 m above the slope) |
| A4b launch, no GNSS | … 17.5:3 … | lift-off 13.2 | 4.30 s |
| A4c launch, navigation only | 29.8:3 | lift-off 13.2 | 16.53 s (no ground decision possible before the run; Unknown → Flying by the single-source rule) |
| A5 aborted takeoff (7 m/s run, stops) | 5.0:1 20.8:2 26.0:1 | ground | candidate, LiDAR veto, **aborted** (count 1) |
| A6 cruise / A7 thermal 25° bank / A8 descent 500 → 50 m | 5.0:3 | air | Flying from start-up, nothing else |
| A9 ridge soaring 8–15 m, 1–4 m/s ground speed, 240 s | 5.0:3 6.0:4 | air | Approach; **never LandingCandidate** |
| A10–12 approach, flare, touchdown, stand | 5.0:3 79.8:4 104.6:5 109.6:6 | touchdown 104.4 | Approach 24.6 s before touchdown; **Landed 5.2 s after touchdown** |
| A13 landing then second takeoff | … 58.5:5 63.5:6 122.6:2 168.5:3 … | touchdown 49.2; second lift-off at the cliff (~168) | Landed; the 45 s ground run at 9 m/s stays `TakeoffCandidate` (LiDAR veto) until the cliff edge, then Flying |
| A14 switched on in flight | 5.0:3 | air | Flying after the 5 s start-up decision |
| A15 20 s total outage in flight | 5.0:3 | air | **PhaseHeld**, > 1 000 held updates, never landed |
| Hard landing (−4 m/s, no flare) | 5.2:3 6.2:4 12.0:5 17.0:6 | touchdown 9.7 | Landed 7.3 s after touchdown |
| Dropped 1.4 m on the ground | 5.0:1 | ground | no flight |
| Landing, navigation + IMU only | 18.0:3 87.8:5 92.8:6 | touchdown 77.4 | Landed **15.4 s** after touchdown |
| D47 full flight (launch, thermal, glide, land) | 5.0:1 14.0:2 17.6:3 18.6:4 33.8:3 307.8:4 341.7:5 346.7:6 | — | 1 takeoff, 1 landing, 0 Caution+ |
| D55 **REAL pipeline** | 10.5:1 15.2:2 18.8:3 19.8:4 37.5:3 64.4:4 85.4:5 90.4:6 | climb at 15 s, touchdown 82.5 | Flying 3.8 s after climb start; Landed 7.9 s after touchdown |

**Over all scenarios:**
* false transitions in normal synthetic scenarios: **0**;
* missed transitions: **0**;
* oscillations: **none**. The Approach ↔ Flying changes in A4 / D47 / D55 follow real clearance changes
  across the 30 / 45 m hysteresis.

**Recovery after sensor loss:**
* navigation loss 40–70 s: phase held Flying, cleared after the return (D51);
* LiDAR loss 40–60 s: Approach → Flying (exit after 3 s without LiDAR) → Approach on return (D50).

---

## 6. Development findings (what the tests changed)

1. **Navigation-only start-up never decided (A4c, landing without LiDAR / GNSS):** from `Unknown`, one
   airborne source was not enough. Added the same single-source rule as for takeoff (15 s).
2. **`AltitudeSteady` flickered:** it tested the instantaneous \|vz\| (0.12 m/s noise) and restarted the
   dwell timers. It is now judged over the 8 s window (spread + net rate). The navigation-only landing
   delay went from 31.5 to **15.4 s**.
3. **False advisory at start-up:** standing on the launch in `Unknown` (clearance 1 m) raised
   `LowGroundClearance`. `Unknown` now requires the clearance above the 2.5 m ground threshold; a test
   asserts no LowGroundClearance in Unknown / Ground.
4. **Uncertain navigation was silent (C41):** descent warnings were correctly withheld, but nothing said
   why. `NavigationDegraded` (Advisory) now also covers "velocity σ > 1 m/s while airborne".
5. **Step threshold 25 % was too coarse:** a 4 m obstacle at 32 m was not flagged. Changed to
   max(1.5 m, 10 %) = 10σ of the 1 % range noise. The rate's residual check already prevented fake
   closure.
6. **Test / scenario corrections (recorded for transparency):**
   * time-window sampling bugs in two tests;
   * scenario arithmetic in the full flight and the real-pipeline profile (the pilot never reached the
     ground);
   * an inconsistent landing profile;
   * optimistic bounds that did not account for 1 % range noise.

   None of these changed algorithm behaviour.

---

## 7. Warning results

### 7.1 Detection, escalation, clearing (held-out seed 101)

Event codes: type / transition / severity; types 2 ExcessiveDescent, 3 LowClearance, 4 RapidClosure,
5 HazardousApproach, 6 TerrainUnavailable, 7 NavigationDegraded; transitions 1 Raised, 2 Escalated,
3 Deescalated, 4 Cleared.

| Scenario | Events | Truth / result |
|---|---|---|
| C31 controlled descent −1.5 m/s, 300 → 120 m | none | correct |
| C32/37 normal approach + flare + touchdown | 94.4 LowClearance Advisory raised, 106.6 cleared | only the informational advisory; no Caution+ |
| C33 −7 m/s then −12 m/s then recovery | 21.0 Raised Warning, 35.0 Escalated Critical, 52.0 Cleared | **Warning 1.01 s** after −7 began; **Critical 0.01 s** after −12; **cleared 2.01 s** after recovery; one episode |
| C34 −5 m/s without LiDAR | 1.0 Descent Caution; 8.0 TerrainUnavailable **Caution** | no HazardousApproach (not confirmed without LiDAR); one TerrainUnavailable event |
| C35/38 level flight into terrain rising 0.6 | 6.5 Rapid Warning, 8.2 Rapid Critical, 8.2 Hazardous Warning, 9.2 Hazardous Critical | truth TTC < 4 s at 6.17 s (Warning **+0.38 s**); TTC < 2 s at 8.17 s (Critical **+0.04 s**); truth clearance < 6 m at 9.17 s (Critical **1.0 s earlier**) |
| C36 low clearance then climbing | 1.0 LowClearance Advisory, 39.1 cleared | advisory only |
| C39 LiDAR dies on final (≈30 m) | 83.0 TerrainUnavailable Advisory | 3.0 s after the loss; no invented hazard |
| C40 50° wingovers at 25 m, attitude degraded | none | no false hazard |
| C41 −7 m/s, navigation velocity σ 1.5 | 7.0 NavigationDegraded Advisory | descent **not confirmed** (σ above the limit); the reason is published |
| C43 sink oscillating 3.2–3.8 around 3.5 | one Raised Caution | **no toggling** (one episode, no clear) |
| C44/45 −3 → −11 → −7 → −1.5 m/s | Raised C, Escalated W, Escalated Crit, Deescalated W, Cleared | full sequence in **one episode** |
| C46 caution, repeat within cooldown, then Critical | 11.0 Raised, 17.0 Cleared, (repeat suppressed), 30.5 Raised Critical | repeat suppressed (`suppressedReactivations` ≥ 1); Critical announced during cooldown |
| 60 s continuous Warning | 1 event | no per-frame flood |
| D52 −6.5 m/s at 12 m/s into rising terrain | ExcessiveDescent, Rapid, Hazardous, LowClearance overlapping | mask 0x0f (4 warnings), highest Critical |
| Contradiction: navigation −6 m/s vs LiDAR steady 20 m | ExcessiveDescent raised | no terrain hazard (LiDAR closure wins) |
| Stale navigation (2 s old) | none | not used as evidence |

### 7.2 False positives and missed hazards

* **False positives:** 20 normal flights (thermal + landing, 10 held-out seeds) gave **0 Caution-or-higher**
  and 10 advisories (one per landing). Fault-injection flights (IMU, barometers, UKF degraded, GNSS,
  LiDAR, +30 m altitude jump) gave **0 Warning-or-higher**.
* **Missed synthetic hazards:** **0** among the hazard scenarios above (C33, C34, C35/38, D52 and the
  contradiction case).
* **Duplicate suppression:** every input delivered twice (D53) gave the **same 12 events** as the clean
  run; 9 091 LiDAR duplicates were skipped.

---

## 8. Integration and fault tolerance

| Test | Result |
|---|---|
| D47/48 full synthetic flight | sequence Ground → Candidate → Flying → (Approach) → LandingCandidate → Landed; 10 events; 0 Caution+; all events `Unvalidated` |
| D49 GNSS lost, LiDAR weak for 210 s, navigation and attitude degraded | one NavigationDegraded event; still lands; no Warning+ |
| D50 LiDAR loss / recovery at 20 m | TerrainUnavailable raised 3.0 s after the loss, cleared on return |
| D51 navigation unavailable 30 s | NavigationDegraded **Caution** raised and cleared; no descent warning; never landed |
| D53 duplicates | identical decisions |
| D54 monitor → 4-slot queue, no consumer | Critical kept; sink-reported drops = queue drops |
| D55 REAL pipeline | full phase sequence; 8 phase notices popped from the bounded queue; standing clearance 1.0 ± 0.3 m; `ModelPlaceholder`, `ImuMountingUnverified`, `LidarMountingUnverified` propagated |
| D56 2 h soaring | 1 phase change, ≤ 2 events, 720 001 updates |
| Faults: IMUs / barometers / UKF / GNSS / LiDAR / +30 m altitude jump | never LandingCandidate / Landed; no Warning+; invalid values NaN |
| Queue: 2 000 mixed events, 20 criticals, consumer 10× slower | dropped new 1 579, dropped queued 234, **dropped critical 0** |

---

## 9. Parameter sweeps (tuning seeds 1–3, defaults frozen afterwards)

**Runs per value:** full flight, carried device, low soaring, and sink oscillation then −7 m/s; 12 runs
per value.

| Parameter | Value | Takeoff delay | Landing delay | Missed T/L | False Flying / Landed | Caution+ (normal) | Descent Warning delay |
|---|---|---|---|---|---|---|---|
| takeoffConfirmMs | 1 000 / **3 000** / 6 000 | 2.32 / **4.32** / 7.32 s | 5.14 s | 0/0 | 0 / 0 | 0 | 0.51 s |
| landingConfirmMs | 2 000 / **5 000** / 10 000 | 4.32 s | 2.14 / **5.14** / 10.14 s | 0/0 | 0 / 0 | 0 | 0.51 s |
| descent raiseMs | 200 / **1 000** / 2 000 | 4.32 s | 5.14 s | 0/0 | 0 / 0 | 0 | 0.51 / **0.51** / 1.23 s |
| descent hysteresis | 0 / **0.5** / 1.0 m/s | 4.32 s | 5.14 s | 0/0 | 0 / 0 | 0 | 0.51 s |

**Trade-offs:**
* Shorter confirmation gives earlier phases. On these runs no false transition appeared even at 1 s or
  2 s, so the held-out margin is not established. The 3 s / 5 s defaults were kept as conservative
  values against unmodelled real motion (turbulence on launch, walking with the wing).
* The descent persistence of 2 s delays the Warning by 0.7 s.
* **Limitation of this sweep:** the tuning set does not discriminate the descent hysteresis. The
  persistence timers alone prevent toggling here. The held-out test C43 checks no-toggling with the
  default.

---

## 10. Performance and resources

| Item | Result | Basis |
|---|---|---|
| CubeIDE Debug | 0 errors / 0 warnings; text 122 804 (+20 496), data 104, bss 51 120 (+3 088) | headless build of a copy |
| CubeIDE Release (`-Os`) | 0 errors / 0 warnings; text 70 920 (+11 584), data 104, bss 51 112 (+3 088) | same |
| Release safety code | 9 500 B | `nm -S` |
| Static RAM | SensorProcessingTask 18 352 → 21 160 B; HealthMonitorTask 3 104 → 3 300 B; `g_systemHealth` 672 → 756 B | `nm -S` |
| Object sizes (Cortex-M7) | SafetyMonitor 1 896, GroundClearanceEstimator 772, FlightStateDetector 336, LandingSafetyAnalyzer 556, SafetySnapshot 188, SafetyInput 140, SafetyEvent 40, SafetyEventQueue<16> 680, SafetySnapshotMailbox 196 B | `sizeof` for ARM |
| Heap | 0 | code review |
| Stack | `Run` 256 → 296 B; safety chain ≈ 0.4 KB; deepest chain (navigation) ≈ 968 B of 4 096 B | Release `.su` |
| Strict ARM compile (`-Wconversion -Wdouble-promotion -Wfloat-conversion -Wcast-align -Wshadow -Wpedantic`, O0 / O2) | 0 warnings in the new code | arm-none-eabi-g++ 14.3 |
| Host time | 126 ns per `SafetyMonitor::update` (best of 5 × 60 000 frames, LiDAR valid most frames) | Apple M4, `-O2` |
| Target time | **estimate 40–130 µs** per update (≤ 1.3 % of 10 ms); attitude + navigation + safety ≤ ~3.6 % | Milestone 3 ratio; not measured |
| Event-queue loss | depends on the consumer; none exists yet in firmware (§7.2 of the design) | — |

---

## 11. Regression and sanitizers

| Run | Result |
|---|---|
| Host, no sanitizer (`-O1 -Wall -Wextra -Wpedantic -Wshadow -Werror`) | **393 / 393** |
| AddressSanitizer + UndefinedBehaviorSanitizer (`SANITIZE=1`) | **393 / 393, 0 reports** |
| ThreadSanitizer (`SANITIZE=thread`; includes the mailbox concurrency tests) | **393 / 393, 0 reports** |

All 338 earlier tests are unchanged and still pass. No existing test was weakened.

---

## 12. Not validated / needs hardware

1. **LiDAR mounting and boresight** relative to the body, its height above the ground when standing, and
   behaviour over grass, water, snow and in sun (signal strength).
2. **Real flight data:**
   * all phase thresholds (ground speed, stillness, steadiness, dwell);
   * all warning thresholds and persistence;
   * real pilot motion on launch and after landing (walking with the wing).
3. **Target timing, stack high-water mark** and critical-section duration.
4. **Terrain:** slopes, vegetation and buildings in real approaches (single-beam limitations).
5. **Event consumers** (audio / LED / logging), and their queue sizing.
6. **Safety:** none of this has been reviewed or validated as a safety function. It does not replace
   pilot judgement or approved equipment.
