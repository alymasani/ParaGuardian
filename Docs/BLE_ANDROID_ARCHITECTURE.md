# BLE and Android architecture

Milestone 7 connects the M6 emergency manager to a bounded packet transport and adds an Android companion. The full emergency workflow runs in simulation. Production device traffic stays disabled until the CYW20822 packet backend and peer authentication are provisioned. Phone-originated, user-confirmed SMS can work independently of the MCU.

## Repository baseline and protocol inventory

The source uses STM32H743ZIT6, HSI at 64 MHz, STM32CubeH7, CMSIS-RTOS2, FreeRTOS 10.6.2 and C++17. The local SoarOS checkout is at `426c1efe895bb58ba2a57ea128c9aad642e9be9f`. Prior milestone documentation and the observed tests agree on the emergency pipeline. Sensor processing publishes crash and safety snapshots. EmergencyAlert runs above processing and below acquisition, with a 20 ms idle wait. It owns the incident state, outputs, and the SOS snapshot separately from the events queue.

| Existing representation | Ownership and compatibility |
| --- | --- |
| `SosRequest` v1 alert and cancel | Explicit 64-byte encoding. Copied byte for byte into a BLE envelope. |
| `EmergencyCommand` | Internal MCU structure with a structural check word, source-local sequence and MCU receipt time. BLE carries a one-byte opcode and header incident ID. |
| `TransportReport` | Internal MCU structure. Correlates incident and request revision. Link changes and write results come from the bridge or driver. Phone reports use three explicit wire bytes. |
| `EmergencyPersistRecord` | Existing local versioned persistence format. Production still uses `NullEmergencyStore`. |
| `SosPayload`, `formatSosRecord` | Older compact ASCII projection with no incident acknowledgment. Remains available to its existing callers. |
| `EzSerialLink` | Bounded UART text parser and command-response timer. It provides no verified CYW20822 event map, GATT provisioning, or peer trust. |

The v1 SOS payload uses little endian. Signed coordinates, heights and velocity use two's complement. CRC-32/IEEE covers bytes 0 through 59. These are byte offsets from `SosRequest.cpp`.

| Offset | Width | Field |
| ---: | ---: | --- |
| 0 | 1 | Version, 1 |
| 1 | 1 | Kind, 1 alert or 2 cancellation notice |
| 2 | 1 | Incident trigger |
| 3 | 1 | MCU emergency state |
| 4 | 4 | Incident ID |
| 8 | 2 | Revision |
| 10 | 2 | Validity flags |
| 12 | 4 | Request creation uptime in ms |
| 16 | 4 | Event uptime in ms |
| 20, 24 | 4 each | Latitude and longitude in 10⁻⁷ degrees |
| 28 | 4 | GNSS height above MSL in dm |
| 32 | 2 | Horizontal accuracy in dm |
| 34, 35 | 1 each | Fix type and satellite count |
| 36 | 4 | Fix age in ms, 0xFFFFFFFF for absent fix |
| 40 | 4 | Relative barometric altitude in dm |
| 44 | 2 | Vertical speed in cm/s |
| 46, 48 | 2 each | Peak acceleration in centi-g and peak angular rate in degrees/s |
| 50, 51 | 1 each | Impact class and count |
| 52 | 4 | Crash evidence bits |
| 56 | 2 | Battery in mV, 0 when unknown |
| 58 | 2 | Low 16 device-health bits |
| 60 | 4 | CRC-32/IEEE |

Validity bits 0–9 are position, MSL height, relative altitude, vertical speed, impact, saturated impact, battery, stale position, volatile incident ID and restored incident. Incident IDs start at 1, increment for an opened incident, skip 0 on wrap and can restart after reset without storage. A retry within a cycle keeps its revision. Rebuilding a request for recovery or cancellation increments that revision. Production authorization requires an incident namespace that remains unique across boots.

## Packet and GATT profiles

The planned device is the peripheral and GATT server. Android is the central and GATT client. The CYW20822 can implement the roles with a supported module image. The exact loaded firmware and its configuration remain a board bring-up dependency. [CYW20822 data sheet](https://www.infineon.com/dgdl/Infineon-CYW20822-P4TAI040_CYW20822-P4EPI040_BLUETOOTH_R_LE_MODULE-DataSheet-v04_00-EN.pdf?fileId=8ac78c8c8929aa4d018a1ede71db1ef1) · [CYW20822 EZ-Serial guide](https://www.infineon.com/dgdl/Infineon-EZ-Serial_firmware_platform_user_guide_for_CYW20822_module-UserManual-v02_00-EN.pdf?fileId=8ac78c8c8d2fe47b018e17ad650a6ea6)

The service UUID is `5d124b00-72b1-419f-b8f7-5d477dc613cb`. Replace the last byte of the first UUID group as shown below. Advertising exposes the service UUID. The authenticated characteristics carry operational data.

| UUID byte | Channel | Properties and direction | Access |
| --- | --- | --- | --- |
| 01 | SOS and cancellation notices | Indicate, device to phone | Authorized encrypted association |
| 02 | Emergency state, health, navigation, safety and telemetry | Notify, device to phone | Authorized encrypted association |
| 03 | Phone reports and commands | Write with response, phone to device | Authorized encrypted association |
| 04 | Device identity and capabilities | Reserved read for provisioning | Define access with the hardware trust mechanism |

Routine fields share a coalesced status characteristic. The two device-to-phone characteristics share sequence numbering and one reassembly slot, so the backend must serialize their fragments in emission order. Status can be abandoned when a higher-sequence SOS begins. Time synchronization is reserved for a later extension. Phone wall time in a message is estimated from receipt time and the difference between device event and creation uptimes. The estimate excludes unmeasured queue and radio delay.

| BLE envelope offset | Bytes | Meaning |
| ---: | ---: | --- |
| 0 | 1 | Envelope version, 1 |
| 1 | 1 | Type, 1 SOS, 2 report, 3 command, 4 status, 5 reserved telemetry |
| 2 | 4 | Nonzero sequence per direction and authenticated session |
| 6 | 4 | Incident ID, zero allowed for general status and applicable commands |
| 10 | 2 | Payload length, maximum 96 |
| 12 | Variable | Payload |
| 12 + length | 4 | CRC-32/IEEE over header and payload |

A fragment starts with the envelope sequence (4 bytes), byte offset (1) and total envelope size (1), followed by contiguous data. Maximum envelope and fragment sizes are 112 and 118 bytes. At ATT MTU 23 the attribute value has 20 bytes and carries 14 envelope bytes per fragment. An 80-byte SOS envelope takes six fragments. An ATT value limit of at least 86 can carry it in one. Android requests an MTU, uses the negotiated result and caps attribute values at 118 bytes. Android 14 and later may request 517 from the platform for the first client regardless of the application's request. [Android BluetoothGatt.requestMtu](https://developer.android.com/reference/android/bluetooth/BluetoothGatt#requestMtu(int))

One assembly is permitted per direction. It expires two seconds after the first fragment. Gaps are rejected without retaining out-of-order fragments. Exact duplicates are tolerated, and complete sequences at or below the high-water mark are dropped. A new session clears state. The packet port must flush old traffic and change generation before authorizing a new session. Unsupported versions fail closed. Each endpoint uses explicit widths and byte order. Shared vectors cover alert, acceptance and cancel.

Phone report payload is one stage byte and a little-endian revision. Wire values are 5 stored by phone, 6 phone rejects, 7 send attempted, 8 delivery confirmed, 9 failure, 10 cancellation recorded, 11 delivery unknown. Values 1–4 are internal link/write reports. A phone command payload carries 1 manual SOS, 2 cancel current countdown, 3 acknowledge alert or 4 request status. No remote reset is exposed. Cancellation uses the exact current incident and MCU arrival time. The app displays the resulting device state after requesting cancellation.

The 40-byte status payload starts with state, trigger, stage, failure, attempts and cycles (six bytes). Reported stage bits are at offset 6 (u16), countdown at 8, retry delay at 12 and flags at 16 (u32). Bytes 20–25 are cancellation notice, flight phase, navigation state, warning mask, validity and a reserved zero. Validity bits 0–2 cover altitude, vertical speed and the safety summary. Battery (u16 mV) is at 26, signed relative altitude (dm) at 28, signed vertical speed (cm/s) at 32 and quality flags at 36.

## Firmware scheduling and recovery

EmergencyAlert owns the bridge, reassembly, pending output and manager. There are zero BLE operations in sensor tasks. Each wake admits at most four incoming fragments and attempts one outgoing fragment. The port must copy or refuse immediately. SOS can displace a routine status. The newest routine status is encoded about twice per second or at a state change. There is one output slot and one reliable M6 incident snapshot. Diagnostics are published through a critical-section-protected latest-value mailbox.

Local queueing, GATT indication confirmation, durable phone acceptance and provider delivery are distinct events. The bridge accepts phone reports only for the current fully queued request and its revision. Confirmation requires prior phone acceptance and a send attempt. A deployed driver could publish a genuine transfer result when supported. The current adapter reports queue activity and uses application-level acknowledgments for phone outcomes.

Offline waiting consumes no attempts. Default policy permits eight attempts per cycle, a 10-second phone acceptance deadline, exponential delays from 2 to 60 seconds and up to three total automatic cycles. A disconnect during an attempt consumes that attempt. On reconnect an unresolved acceptance or outcome can start another bounded cycle under the same incident ID and next revision. Exhaustion is visible as an unresolved incident, distinct from a phone-reported failure. After phone acceptance, 120 seconds without an outcome yields unknown. Valid late outcomes can still update eligible incidents. Explicit local reattempts follow the existing M6 behavior. Since the phone uses an idempotency key, a later revision retrieves or updates the record without resending the same incident.

## Trust and data handling

The transport accepts a physical session only through a hardware-backed verification boundary. The STM32 uses `UnconfiguredPacketPort` and Android supplies a verifier that admits no physical peers. The two simulation endpoints are opt-in and use a separate route and randomized simulator identity. A device's address, name, Android CompanionDeviceManager association and bond state alone cannot grant access.

For production, pair in a physical access window with authenticated LE Secure Connections or a reviewed provisioned mechanism. Pin the authenticated device and companion identities. Authorize each encrypted connection before enabling sensitive reads, indications and writes. Use a secure association-derived session at both ends with a new packet generation after reconnect. Clear queues and enforce monotonic sequences within it. Application-level credential proofs, if needed by the module configuration, require a reviewed protocol with platform cryptographic APIs and secure random challenges. The CRC serves corruption detection. Tokens and keys are absent in this build. Re-pairing and lost-phone replacement must revoke the old identity and remove its bond before allowing enrollment. [Hardware integration](BLE_ANDROID_HARDWARE_INTEGRATION.md)

SQLite schema v1 uses `(device, incident)` as its primary key. Android stores the record before reporting acceptance. It records the attempt before calling the mock provider. Surviving attempts are recovered as unknown to avoid duplicate sends. Report codes are reconstructed from durable stages and replayed when the same incident is received. Read/write failures result in a diagnostic and no acceptance for a failed commit. The app uses private storage, disables backup and excludes locations and contact data from exported diagnostics. Raw payload and location are redacted after 30 days or on request. Minimal deduplication rows survive historical redaction until app data is cleared. Contacts and preferences use private `SharedPreferences`. There are currently no app-managed production credentials. Any future credentials belong in Android Keystore-backed storage.

## Delivery, Android monitoring and location

The simulator can return provider acceptance, simulated delivery, explicit failure or unknown. The physical route prepares one explicitly requested SMS composer per approved contact. Opening a composer is recorded as unknown. The pilot reviews and presses Send in the SMS app. A phone-originated manual SOS works independently of device monitoring. Automatic physical delivery requires a configured provider and verified status semantics. No production credentials, direct SMS permission, server or automated external send are included. [Android common intents](https://developer.android.com/guide/components/intents-common#SendMessage)

Setup uses Android’s companion picker. It filters by the service UUID, then saves an association for known-device discovery. The foreground service starts from the visible activity and uses the connected-device type with an ongoing notification and Stop action. The GATT client serializes MTU negotiation, discovery, subscriptions and writes. A step times out after 12 seconds. Reconnect delay grows from 1 to 60 seconds. Bluetooth-off waits for adapter availability, and Bluetooth permission loss ends monitoring. App launch and reboot start with monitoring inactive. There is no automatic restart receiver or CompanionDeviceService. The app may continue monitoring after the UI is dismissed while the service lives. A user stop, force stop or process death ends GATT. [Background BLE](https://developer.android.com/develop/connectivity/bluetooth/ble/background) · [Companion device pairing](https://developer.android.com/develop/connectivity/bluetooth/companion-device-pairing) · [Foreground service changes](https://developer.android.com/develop/background-work/services/fgs/changes)

API 31+ uses Bluetooth runtime permissions, API 33+ asks for notification permission and API 34+ declares the connected-device permission and service type. The app targets API 36. It uses no background location permission and no fullscreen activity launch. A recent device GNSS fix is preferred. For a visible emergency with insufficient device data, the app can request one phone fix with a short deadline, then consider its last known fix. If the UI is absent it can use a timestamped fix already acquired while visible, subject to its reported age. Both routes can proceed with location unavailable. Source, estimated fix timestamp, age, reported accuracy and newly-acquired status remain attached to the selected fix. Stale positions are labeled in the preview and outgoing text.
