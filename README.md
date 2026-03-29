# ARA Backend — Autonomous Racing AI Coach

Live telemetry visualization server for autonomous racing data. Streams real-world telemetry from Yas Marina Circuit directly into [Foxglove Studio](https://foxglove.dev), overlaying the optimal racing line onto the live camera feed.

## What it does

- Streams the front-left camera feed from real MCAP race data
- Projects the good lap trajectory onto the camera image in real time (3D → 2D using camera intrinsics)
- Renders a 3D scene with car positions and track boundaries
- Publishes camera calibration for metric overlays in Foxglove

## Requirements

```
Python 3.10+
foxglove-sdk
mcap-ros2-support
scipy
opencv-python
pyyaml
numpy
```

Install with:

```bash
python -m venv venv
source venv/bin/activate
pip install foxglove-sdk mcap-ros2-support scipy opencv-python pyyaml numpy
```

## Data files

Place these in the project root (not included in the repo due to size):

| File | Description |
|------|-------------|
| `hackathon_fast_laps.mcap` | Two fastest laps — used as the coach / reference |
| `hackathon_good_lap.mcap` | Conservative lap — used as the student reference line |
| `hackathon_wheel_to_wheel.mcap` | Multi-car competitive race scenario |
| `yas_marina_bnd.json` | Yas Marina circuit boundary polygons |

On first run, `server.py` rips state estimation data from the MCAPs into `.npy` files to speed up subsequent starts.

## Running

```bash
source venv/bin/activate
python server.py
```

Connect Foxglove Studio to `ws://localhost:8765`. Add a **3D panel** on topic `/scene` and an **Image panel** on topic `/camera`.

To allow other devices on the network to connect, the server binds to `0.0.0.0` — use `ws://<your-ip>:8765`.

## How the projection works

For each frame:

1. The car's current pose (`x, y, z, yaw`) is read from the fast lap state estimation.
2. Good-lap trajectory points within 100 m ahead are selected.
3. Points are transformed: **world → body frame → camera frame** using the extrinsic constants at the top of `server.py`.
4. `cv2.projectPoints` applies the real camera intrinsics and distortion from `intrinsics/camera_fl_info.yaml`.
5. The projected polyline is drawn onto the decoded JPEG before sending.

### Tuning the projection

If the green line is misaligned, adjust the extrinsic constants near the top of `server.py`:

```python
CAM_T_BODY = np.array([0.5, 0.3, 0.8])  # camera position in body frame (forward, left, up) in metres
_PITCH_DEG  = -12.0                       # negative = tilt down
_YAW_DEG    = -5.0                        # negative = tilt left
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera` | `CompressedImage` | Front-left camera with good-lap overlay |
| `/camera_info` | `CameraCalibration` | Intrinsics for `camera_fl` |
| `/scene` | `SceneUpdate` | 3D scene: car, good lap line, track boundaries |

## Intrinsics

Camera calibration YAMLs are in `intrinsics/`. The server uses `camera_fl_info.yaml` (front-left, 1506×728).

## Dataset

Real telemetry from Yas Marina Circuit — Constructor GenAI Hackathon 2026, Autonomous Track.
Topics include state estimation at ~100 Hz, camera images at ~10 Hz, GPS, IMU, CAN bus, tyre, and suspension data.
See `sd_msgs/` for ROS 2 message definitions.
