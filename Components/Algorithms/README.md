# Components/Algorithms

Intentionally empty. The Madgwick attitude filter, Kalman filter/UKF,
aerodynamic model, crash detection, landing guidance and SOS logic are **out of
scope so far**.

When implemented, algorithms must:

- take their input from the preprocessing layer, i.e. a
  `pg::ProcessedSensorFrame` (Components/Processing/ProcessedSensorFrame.hpp),
  read with `SensorProcessingTask::Inst().frames().read(frame)` (or a future
  per-consumer queue). Use only fields whose `valid` flag is true and check
  `qualityFlags`;
  - Madgwick: `angularRate_radps` (+ `gyroDtMs`/`gyroDtValid`),
    `specificForce_mps2`, optionally `mag.fieldBody_uT` (body FRD frame);
  - UKF: `baroAltitude_m`, `gnss`, `lidar` (+ vertical acceleration derived
    from the attitude filter; gravity is NOT removed by preprocessing);
- convert body (FRD) / NED quantities to ENU only through `pg::nedToEnu()`;
- never call HAL or driver functions directly;
- live in portable C++ so they can be unit-tested on a laptop like the drivers.
