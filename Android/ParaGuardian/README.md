# ParaGuardian companion

Use Java 17 and Android SDK Platform 36. The Gradle wrapper pins 8.11.1. Open this directory in Android Studio or run the following from this directory.

```sh
./gradlew :app:assembleDebug :app:testDebugUnitTest :app:lintDebug
./gradlew :app:connectedDebugAndroidTest
```

The C++ fixture runs the real crash detector and emergency bridge. To include it in JVM tests, invoke from the repository root.

```sh
sh Tests/build_ble_peer.sh /tmp/paraguardian_ble_peer
cd Android/ParaGuardian
PG_BLE_PEER=/tmp/paraguardian_ble_peer ./gradlew :app:testDebugUnitTest --rerun-tasks
```

The app starts in Simulation. Connect the simulator in Setup. Open Emergency, select **Flight**, **Suspect crash** and **Advance 31 seconds**. Confirm that the simulated delivery is visible in the phone record. Use **Repeat SOS** to verify the same incident is reused. Fault controls can drop or delay reports, corrupt the next SOS, remove GNSS or network, reopen incident storage and change the simulated provider response.

Physical mode offers Android device association and GATT setup. A provisioned hardware transport and authenticated peer verification are required to carry incident traffic. The independent phone SOS path prepares an SMS for an approved contact. Send is confirmed inside the phone’s messaging app. The app reports delivery unknown after opening the composer.

## Milestone 8: unattended delivery and lifecycle recovery

Settings → **Unattended emergency delivery** stores an emergency-server URL, device id and per-device secret. The
secret is encrypted with an Android Keystore key. Enabling it routes physical incidents (route `INTERNET`) through
a durable outbox to the server (`Backend/`). The server signs, de-duplicates and delivers to consented contacts.
The app shows these stages:
- submission pending
- emergency server accepted
- provider accepted
- delivered (a provider report)
- failed
- unknown

The simulation can optionally submit **test** incidents to the server. Test incidents never reach a live provider.
Release builds accept only HTTPS. Debug builds also allow `http://10.0.2.2` / `127.0.0.1` for the local test
backend: `PG_HOST=0.0.0.0` is refused without TLS, so run the backend on the host at `127.0.0.1` and use the emulator
address `10.0.2.2` through `adb reverse tcp:8443 tcp:8443` or an equivalent.

Monitoring recovery:
- the persisted user request is kept separate from the actual service state;
- START_STICKY restart after process death;
- a boot receiver;
- a companion presence service (Android 12+);
- a persisted JobScheduler job for pending submissions.

After a force stop the app must be opened again (Android policy). The UI shows "Monitoring interrupted".

```sh
./gradlew :app:testDebugUnitTest      # includes a Kotlin -> real Python backend end-to-end test (needs python3)
```

[Architecture](../../Docs/BLE_ANDROID_ARCHITECTURE.md) · [Validation and limitations](../../Docs/BLE_ANDROID_VALIDATION_REPORT.md)
