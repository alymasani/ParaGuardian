# Milestone 7 validation

Tested on November 18, 2026 using Apple Clang 17, native STM32CubeIDE 2.2.0 with GNU Tools for STM32 14.3, Java 17, Gradle 8.11.1, Android Gradle Plugin 8.10.1, Kotlin 2.1.21 and Android API 36. The Android virtual device was an ARM64 Google APIs image at API 36. The PCB and CYW20822 were unavailable.

## Build and test results

| Check | Result |
| --- | --- |
| Original host suite, strict flags with ASan and UBSan | 463 / 463 passed before edits |
| Final host suite, strict warnings as errors | 473 / 473 passed |
| Final host suite, ASan and UBSan | 473 / 473 passed, no sanitizer findings |
| Final ThreadSanitizer | 473 / 473 passed, no ThreadSanitizer findings |
| CubeIDE native Debug and Release | Both baseline and final built with 0 errors and 0 warnings in clean temporary copies |
| Android debug Kotlin and APK | Built successfully with target SDK 36 |
| Android JVM tests | 14 passed, including shared vectors and the native C++ peer |
| Android 16 connected tests | 4 passed, 0 skipped |
| Android lint | Succeeded with 0 errors and 16 warnings. Warnings are dependency version suggestions, API-specific backup attributes and KTX style suggestions. |
| Actual app force-stop and relaunch on emulator | Different process IDs. The same simulated delivery and stored attempt timeline appeared in History. |

The last Kotlin cross-language test spans native process start, a deterministic crash, SOS round trip and a reconnection. The final JVM suite reported 0.095 seconds, including 0.046 seconds for the cross-language scenario. These are host measurements. No automated check sent an SMS, called anyone, opened a live provider, or contacted a person or emergency service.

## Test coverage and boundaries

- The ten new C++ cases cover golden vectors, multiple ATT sizes, mismatched versions/lengths, inner reassembly, timestamp wrap, missing and malformed packets, replay, unverified sessions, incident and revision matching, stage provenance, backpressure, exhausted retries, reconnect and reset with a null store. A stress case runs 2,000 messages. The full suite retains all 463 prior scenarios, including synthetic flight and emergency-task concurrency tests.
- Kotlin reconstructs the same SOS, accepted report and cancel command bytes. The shared file contains 14 records, including default-ATT fragments. JVM tests also cover authorization, cancellation before and after dispatch, unsupported versions, missing GNSS, location age, contact consent, network and provider errors, and storage failure.
- The native simulator runs the actual flight/safety and crash algorithms, M6 countdown, M7 C++ bridge and fragmentation. Kotlin reassembles the incident, commits it to a host test file, and records the deterministic mock provider result. All acknowledgments are deliberately dropped. After reconnect, a newly opened store reconstructs the prior outcome. Phone acceptance, attempt and simulated delivery go back to the native firmware. Its manager reaches Delivered for the same incident and the mock provider count is one.
- The separate Android UI integration runs the in-app device simulator through the production Kotlin packet path and SQLite. It shows countdown and simulated delivery in Compose. Instrumentation also tests database reopening with an interrupted attempt, preserves the transition to unknown, checks data redaction, exercises activity recreation and covers notification permission denial. An ADB force-stop/relaunch confirmed that persisted delivery and attempt history survived process termination.
- Pending incidents survive retries independently of the event queue. An MCU reboot with the production null store loses an in-memory SOS and may repeat incident IDs. A separate physical authentication and provisioning check is required before production can accept real device traffic.
- Physical GATT security, screen-off BLE on actual handsets, RF range, BLE coexistence, adapter and SIM behavior, direct SMS permissions and real delivery receipts remain unverified. Emulator tests exercise Kotlin, Android UI and storage. The emulator does not stand in for a hardware peripheral.

## Linked firmware resource changes

| Native section | Debug baseline | Debug M7 | Release baseline | Release M7 |
| --- | ---: | ---: | ---: | ---: |
| Text | 156,636 B | 162,900 B (+6,264) | 89,080 B | 92,088 B (+3,008) |
| Data | 104 B | 104 B | 104 B | 104 B |
| BSS | 59,288 B | 59,960 B (+672) | 59,288 B | 59,952 B (+664) |

EmergencyAlert stack remains 768 words (3,072 bytes). Its compiler-reported static frame is 256 B Debug (+24 B) and 240 B Release (+56 B). Release bridge receive and transmit frames are 64 B and 56 B individually. The bridge reserves 112 B each for outgoing data and incoming assembly, 118 B for a shared packet scratch buffer and 96 B for payload scratch. Existing task/mailbox and nested call costs still apply. Compiler frames do not establish a stack high-water mark.

At ATT 23, one 80-byte SOS envelope requires six output opportunities, about 120 ms before driver, radio and phone delays at the 20 ms idle wake cadence. One negotiated packet needs one opportunity. Each wake admits at most four receive fragments and attempts one transmit fragment. An incomplete assembly or obstructed output expires in 2 seconds. The application acknowledgment timeout remains 10 seconds. Hardware processing time and actual PHY timing were unavailable.

A conservative order-of-magnitude estimate for the CRC work alone is roughly 0.6–1.1 ms for a pathological wake at 64 MHz. It assumes up to four maximum inbound envelopes, one outbound envelope and the inner 60-byte SOS CRC, with 8–14 cycles per CRC bit. Regular acknowledgments are 19-byte envelopes and status is 56 bytes, so ordinary work is smaller. Treat this estimate as an arithmetic model. Cache, flash wait states, UART service, priority interactions and real packet latency require on-target measurements.

## Follow-on acceptance

Milestone 8 should bring up the CYW20822 and provision authenticated peer identities, add reset-safe emergency persistence, and validate actual phone background behavior. If unattended delivery is required, use a legitimately configured provider with idempotency and verifiable delivery status. The current real delivery option is user-confirmed SMS composition. [Hardware sequence](BLE_ANDROID_HARDWARE_INTEGRATION.md)
