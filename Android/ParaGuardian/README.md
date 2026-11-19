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

[Architecture](../../Docs/BLE_ANDROID_ARCHITECTURE.md) · [Validation and limitations](../../Docs/BLE_ANDROID_VALIDATION_REPORT.md)
