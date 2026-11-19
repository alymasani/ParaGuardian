# BLE and Android hardware integration

## Determine the exact module and image

Record the full CYW20822 module suffix, image and version, programming method and schematic revision. The board project does not assign a CYW20822 UART, connection pins or a verified GATT backend. Never infer pin assignments from examples for a different module. Infineon describes both CYSPP serial mode and application/custom GATT possibilities. The M7 Android app expects the custom ParaGuardian service. CYSPP compatibility would require an explicit wrapper and corresponding Android profile change. Confirm the supported custom GATT mechanism against the module and loaded image before selecting it. [Module product and documents](https://www.infineon.com/part/CYW20822-P4EPI040) · [CYW20822 EZ-Serial guide](https://www.infineon.com/dgdl/Infineon-EZ-Serial_firmware_platform_user_guide_for_CYW20822_module-UserManual-v02_00-EN.pdf?fileId=8ac78c8c8d2fe47b018e17ad650a6ea6)

| Verify on schematic or bench | Needed information |
| --- | --- |
| Power and signal levels | Exact supply and UART I/O voltage, ground, power-up and reset order, antenna keepout and selected module/antenna |
| MCU transport | Selected STM32 UART instance and pin AF assignments, module RX/TX, baud/rate framing, RTS/CTS, flow control and overflow |
| Module sideband | Reset, programming recovery, connection and mode lines where present. Record polarity and startup states. |
| Host firmware | Board-specific vendor command map or custom bounded module/MCU packet bridge. Verify RX/TX events, MTU reporting and permissible backpressure. |
| Storage | Persistent incident ID and SOS checkpoint via W25Q128. Exercise resets and concurrent logger traffic. |
| Local alerts | Confirm speaker I²S/SAI, status/cancel input and LED wiring. Verify audibility during radio traffic and battery sag. |

## Replace the simulated packet boundary

1. Verify the module supply and reset with a current-limited supply. Capture boot output and firmware identification using the board's UART or manufacturer programming interface.
2. Provision the service UUID and attributes in the architecture document. Configure indications with CCCD for SOS, notifications for status and write-with-response for phone messages. Reserve identity and capabilities for authenticated provisioning. Validate security on reads, descriptors and writes.
3. Implement `PacketPort` for the chosen module image. Have interrupts or a lower-priority owner feed bounded RX and TX queues. EmergencyAlert consumes at most four RX packets and produces at most one TX packet per wake. Return Busy immediately when the module cannot take another packet. Preserve packet boundaries and ordering between indications and notifications.
4. Publish session generation, authentication result and negotiated payload limit only from verified module events. Clear both queues across disconnect, trust loss or identity change. Never publish Associated based on a connection event alone.
5. Implement the Android `PhysicalTrust` verification boundary. Provision a pinned device identity with a physical association window. Use authenticated LE Secure Connections with verified MITM protection where supported. A headless unit without suitable I/O needs a provisioned out-of-band scheme or reviewed application authentication before carrying commands or private telemetry.
6. Connect to the hardware only after trust is established at both ends. Review the app’s trust model against a cloned advertiser and a second phone. Confirm that removing the old phone blocks reconnect and clearing/replacing the bond requires physical authorization.
7. Keep device ID and incident identities stable across resets or provide a vetted boot namespace in the authenticated session. Validate flash commit and recovery across brownout before enabling physical alert traffic.
8. Verify independent local sound and LED alerts while BLE is disabled, backpressured or unavailable. Bring up the MCU and BLE with the simulator first and use approved test-only recipient routes for any later external send checks.

## Bench acceptance matrix

| Real phones and operating systems | Actions to run |
| --- | --- |
| Android 8–11, legacy permission path | Companion picker, known-device reconnect, disabled adapter and missing location |
| Android 12 and 13 | Bluetooth runtime permission grant, denial and revocation. Notification denial on 13. |
| Android 14 | Connected-device FGS declaration and start conditions. MTU and indication subscription on actual controller. |
| Android 15 and 16, at least two vendors | Screen off, UI dismissed, OS Stop, force stop, reboot, out of range and backoff. Confirm the UI accurately reports inactive monitoring. |
| All commissioned devices | ATT 23 and large MTU, malformed fragments, session teardown, replayed auth traffic, two phones, revoked phone, no-location SOS, retry exhaustion, restoration and RF/power stress |

Capture module and Android logs with addresses, secrets, contacts and coordinates redacted. Measure UART and radio backpressure, task high-water marks, radio current and the longest run time of a transport wake with a logic analyzer or DWT counter. Exercise location fallback only with synthetic coordinates or consented field data. Physical delivery acceptance requires a configured provider and reliable per-incident outcome semantics. [Android background BLE guidance](https://developer.android.com/develop/connectivity/bluetooth/ble/background)
