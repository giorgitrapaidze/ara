# Constructor GenAI Hackathon 2026 — Multi-Agent & Decision Systems: Autonomous Track Dataset

This dataset contains real telemetry recorded from an autonomous racing car at Yas Marina Circuit, covering a range of driving scenarios from conservative laps to competitive multi-car racing. It is provided as part of the **Autonomous Track** at the Constructor GenAI Hackathon 2026 — Multi-Agent & Decision Systems, giving participants access to the same high-resolution sensor data — GPS, IMU, CAN bus, cameras, tyre and suspension — that a professional race engineering team would work with. Teams are encouraged to use this data to ground their AI coaching features in real-world driving behaviour, validate analysis against actual lap performance, and explore what structured feedback could look like when derived from genuine autonomous racing runs.

## Files

| File | Size | Duration | Date |
|------|------|----------|------|
| `hackathon_good_lap.mcap` | 212 MB | 81.3 s | 2025-11-15 15:13–15:15 UTC |
| `hackathon_fast_laps.mcap` | 191 MB | 74.3 s | 2025-11-15 15:17–15:18 UTC |
| `hackathon_wheel_to_wheel.mcap` | 558 MB | 226.0 s | 2025-11-12 11:24–11:28 UTC |

All files share the same set of topics and message types, with one exception noted below.

## Race Description

- **hackathon_good_lap.mcap** — Example of a slower, conservative lap
- **hackathon_fast_laps.mcap** — Two fastest laps recorded, suitable for lap-to-lap comparison and performance analysis
- **hackathon_wheel_to_wheel.mcap** — Competitive multi-car race scenario

## Topics

### State Estimation

| Topic | Type | Rate |
|-------|------|------|
| `/constructor0/state_estimation` | `StateEstimation` | ~100 Hz |

Fused vehicle state: position (x, y, z), orientation (roll, pitch, yaw), velocity (vx, vy, vz), angular rates, wheel speeds, slip ratios, slip angles, steering angle, pedal inputs (gas, brake, clutch), gear, RPM, brake pressures, and safety flags.

### Cameras

| Topic | Type | Rate |
|-------|------|------|
| `/constructor0/sensor/camera_fl/compressed_image` | `CompressedImage` | ~10 Hz |
| `/constructor0/sensor/camera_r/compressed_image` | `CompressedImage` | ~10 Hz |

Front-left and rear compressed camera images.

### GPS

| Topic | Type | Rate |
|-------|------|------|
| `/constructor0/vectornav/raw/gps` | `GpsGroup` | ~10 Hz |
| `/constructor0/vectornav/raw/gps2` | `GpsGroup` | ~10 Hz |

Raw GNSS data from dual VectorNav receivers.

### Transforms

| Topic | Type | Rate |
|-------|------|------|
| `/tf` | `TFMessage` | ~60 Hz |

Coordinate frame transforms.

### CAN Bus

All CAN topics are published under `/constructor0/can/`.

#### High-Level Control (HL)

Commands sent from the autonomy stack to the BSU.

| Topic | Description | Rate |
|-------|-------------|------|
| `hl_msg_01` | Target brake pressures (per-wheel), throttle, gear | ~111 Hz |
| `hl_msg_02` | Steering (PSA) target control, mode, profile limits | ~111 Hz |
| `hl_msg_03` | DBW enable, ICE enable, push-to-pass, sensor activation flags | ~111 Hz |
| `hl_msg_04` | Additional HL parameters | ~10 Hz |
| `hl_msg_05` | Additional HL parameters | ~154 Hz |
| `hl_msg_06` | Additional HL parameters | ~5 Hz |

Also available as `/constructor0/hl_03` (duplicate of `hl_msg_03`, only in `good_lap` and `fast_laps`).

#### BSU Status & Safety

Feedback from the Base Safety Unit.

| Topic | Description | Rate |
|-------|-------------|------|
| `bsu_status_01` | BSU limp mode, EM stop, ML stop, alive counter, warnings | ~100 Hz |
| `em_status_01` | Emergency manager status, stop deceleration profiles | ~100 Hz |
| `rc_status_01` | Race control flags (session type, car flag, track flag, sector) | ~100 Hz |
| `diagnostic_word_01` | System diagnostics word 1 | ~100 Hz |
| `diagnostic_word_02` | System diagnostics word 2 | ~100 Hz |
| `pd_us_status_01` | PDU status | ~100 Hz |

#### Engine (ICE)

| Topic | Description | Rate |
|-------|-------------|------|
| `ice_status_01` | Gear, throttle, push-to-pass, fuel level, water pressure | ~100 Hz |
| `ice_status_02` | RPM, oil/water temp & pressure, fuel pressure | ~100 Hz |

#### Steering (PSA)

| Topic | Description | Rate |
|-------|-------------|------|
| `psa_status_01` | Actual steering position (rad), speed, torque, current, voltage | ~100 Hz |
| `psa_status_02` | Additional steering status | ~100 Hz |

#### Brakes (CBA — Corner Brake Actuators)

Per-wheel brake actuator feedback.

| Topic | Description | Rate |
|-------|-------------|------|
| `cba_status_fl` | FL actual pressure (Pa, %), target ack, current, voltage | ~100 Hz |
| `cba_status_fr` | FR | ~100 Hz |
| `cba_status_rl` | RL | ~100 Hz |
| `cba_status_rr` | RR | ~100 Hz |

#### Wheel Speeds

| Topic | Description | Rate |
|-------|-------------|------|
| `wheels_speed_01` | All four wheel speeds (rad/s) | ~250 Hz |

#### Kistler IMU & Optical Sensor

High-precision inertial and optical velocity measurements.

| Topic | Description | Rate |
|-------|-------------|------|
| `kistler_acc_body` | Body-frame accelerations (x, y, z) | ~250 Hz |
| `kistler_ang_vel_body` | Body-frame angular velocities | ~250 Hz |
| `kistler_correvit` | Optical ground speed (vx, vy, v, slip angle) | ~250 Hz |
| `kistler_status` | Sensor status | ~1 Hz |

#### MM710 IMU

Secondary IMU (strain-gauge based).

| Topic | Description | Rate |
|-------|-------------|------|
| `mm710_tx1_z_ay` | Z-axis / lateral acceleration | ~200 Hz |
| `mm710_tx2_x_ax` | X-axis / longitudinal acceleration | ~200 Hz |
| `mm710_tx3_y_az` | Y-axis / vertical acceleration | ~200 Hz |

#### Badenia 560 DAQ — Powertrain

| Topic | Description | Rate |
|-------|-------------|------|
| `badenia_560_powertrain_misc` | General powertrain data | ~100 Hz |
| `badenia_560_powertrain_press` | Oil/water/fuel/boost pressure, pushrod strain (front) | ~100 Hz |
| `badenia_560_powertrain_temp` | Oil/water/gearbox temperatures, pushrod strain (rear) | ~100 Hz |

#### Badenia 560 DAQ — Suspension & Ride

| Topic | Description | Rate |
|-------|-------------|------|
| `badenia_560_ride_front` | Front ride height, damper strokes (FL, FR, 3rd) | ~100 Hz |
| `badenia_560_ride_rear` | Rear ride height, damper strokes (RL, RR, 3rd) | ~100 Hz |
| `badenia_560_wheel_load` | Per-wheel loads | ~100 Hz |
| `badenia_560_z_accel_body` | Vertical body acceleration | ~200 Hz |

#### Badenia 560 DAQ — Tyres

| Topic | Description | Rate |
|-------|-------------|------|
| `badenia_560_tpms_front` | Front tyre pressure & temperature (FL, FR) | ~50 Hz |
| `badenia_560_tpms_rear` | Rear tyre pressure & temperature (RL, RR) | ~50 Hz |
| `badenia_560_tyre_surface_temp_front` | Front tyre surface temperatures | ~20 Hz |
| `badenia_560_tyre_surface_temp_rear` | Rear tyre surface temperatures | ~20 Hz |
| `badenia_560_brake_disk_temp` | Brake disc temperatures | ~20 Hz |

#### Badenia 560 DAQ — Misc

| Topic | Description | Rate |
|-------|-------------|------|
| `badenia_560_badenia_misc` | General Badenia miscellaneous data | ~50 Hz |
| `badenia_560_misc4` | Additional misc data | ~10 Hz |
| `badenia_560_misc5` | Additional misc data | ~10 Hz |

## Additional Files

- `sd_msgs/` — ROS 2 message definitions for all custom types
- `yas_marina_bnd.json` — Yas Marina circuit track boundaries
# ara
