import time
import numpy as np
import json
import yaml
import os
import cv2
from scipy.spatial import KDTree
from mcap_ros2.reader import read_ros2_messages

import foxglove
from foxglove.channels import CompressedImageChannel, CameraCalibrationChannel, SceneUpdateChannel
from foxglove.messages import (
    CompressedImage, CameraCalibration, SceneUpdate, SceneEntity,
    CubePrimitive, LinePrimitive, LinePrimitiveLineType,
    Pose, Vector3, Quaternion, Color, Point3, Timestamp,
)

# ---------------------------------------------------------------------------
# Camera extrinsics — camera_fl in base_link body frame
# Body frame: x forward, y left, z up (ROS REP-103)
# Camera frame: x right, y down, z forward (OpenCV convention)
# Tune these if the projection is off.
# ---------------------------------------------------------------------------
CAM_T_BODY = np.array([0.5, 0.3, 0.8])   # camera pos in body frame (fwd, left, up) metres

# Rotation: body → camera (base, no tilt)
#   cam_z = body_x,  cam_x = -body_y,  cam_y = -body_z
_R_BASE = np.array([
    [ 0, -1,  0],
    [ 0,  0, -1],
    [ 1,  0,  0],
], dtype=float)

# Add a small downward pitch (degrees) and slight yaw for front-left offset
_PITCH_DEG = -12.0   # negative = tilt down
_YAW_DEG   = -5.0    # negative = slight left yaw for fl camera

def _Rx(deg):
    a = np.radians(deg)
    return np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]])

def _Rz(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])

CAM_R_BODY = _Rx(_PITCH_DEG) @ _Rz(_YAW_DEG) @ _R_BASE  # body → camera rotation


def rip_trajectory(mcap_path, topic):
    """Rips (x, y, z, yaw, v, gas, brake) from state_estimation topic."""
    print(f"Ripping trajectory from {mcap_path}...")
    traj = []
    try:
        with open(mcap_path, "rb") as f:
            for msg in read_ros2_messages(f):
                if msg.channel.topic == topic:
                    d = msg.ros_msg
                    traj.append([
                        float(d.x_m), float(d.y_m), float(d.z_m),
                        float(d.yaw_rad),
                        float(d.v_mps), float(d.gas), float(d.brake),
                    ])
        print(f"  {len(traj)} state messages.")
        return np.array(traj)
    except Exception as e:
        print(f"Error ripping trajectory: {e}")
        return None


def rip_camera_frames(mcap_path, topic):
    print(f"Ripping camera frames from {mcap_path}...")
    frames = []
    try:
        with open(mcap_path, "rb") as f:
            for msg in read_ros2_messages(f):
                if msg.channel.topic == topic:
                    frames.append(bytes(msg.ros_msg.data))
    except Exception as e:
        print(f"Error ripping camera frames: {e}")
    print(f"  {len(frames)} camera frames.")
    return frames


def load_calibration(yaml_path):
    with open(yaml_path) as f:
        return yaml.safe_load(f)


def make_timestamp(now_ns):
    return Timestamp(sec=int(now_ns // 1_000_000_000), nsec=int(now_ns % 1_000_000_000))


def project_trajectory_onto_frame(
    frame_bgr, world_pts, car_x, car_y, car_z, car_yaw, K, dist_coeffs,
    color=(0, 255, 0), thickness=3,
):
    """
    Projects world_pts (Nx3) onto frame_bgr given car pose and camera intrinsics.
    Draws a polyline in-place. Returns the annotated frame.
    """
    if len(world_pts) == 0:
        return frame_bgr

    # 1. World → body frame  (body: x forward, y left, z up)
    cos_y, sin_y = np.cos(-car_yaw), np.sin(-car_yaw)
    dx = world_pts[:, 0] - car_x
    dy = world_pts[:, 1] - car_y
    dz = world_pts[:, 2] - car_z

    x_b =  cos_y * dx - sin_y * dy
    y_b =  sin_y * dx + cos_y * dy
    z_b =  dz

    pts_body = np.column_stack([x_b, y_b, z_b])

    # 2. Body → camera frame
    pts_cam = (CAM_R_BODY @ (pts_body - CAM_T_BODY).T).T  # (N, 3)

    # 3. Keep only points in front of camera with positive depth
    mask = pts_cam[:, 2] > 0.5
    pts_cam = pts_cam[mask]
    if len(pts_cam) == 0:
        return frame_bgr

    # 4. Project with OpenCV (handles distortion)
    pts_2d, _ = cv2.projectPoints(
        pts_cam.astype(np.float64),
        np.zeros(3), np.zeros(3),
        K, dist_coeffs,
    )
    pts_2d = pts_2d.reshape(-1, 2).astype(np.int32)

    # 5. Filter to within image bounds
    h, w = frame_bgr.shape[:2]
    valid = (pts_2d[:, 0] >= 0) & (pts_2d[:, 0] < w) & \
            (pts_2d[:, 1] >= 0) & (pts_2d[:, 1] < h)
    pts_2d = pts_2d[valid]

    if len(pts_2d) < 2:
        return frame_bgr

    cv2.polylines(frame_bgr, [pts_2d.reshape(-1, 1, 2)], isClosed=False,
                  color=color, thickness=thickness, lineType=cv2.LINE_AA)
    return frame_bgr


def main():
    # -----------------------------------------------------------------------
    # Data Prep — trajectories
    # -----------------------------------------------------------------------
    FAST_NPY = "fast_laps_full.npy"
    GOOD_NPY  = "good_lap_full.npy"

    if not os.path.exists(FAST_NPY):
        data = rip_trajectory("hackathon_fast_laps.mcap", "/constructor0/state_estimation")
        if data is not None:
            np.save(FAST_NPY, data)

    if not os.path.exists(GOOD_NPY):
        data = rip_trajectory("hackathon_good_lap.mcap", "/constructor0/state_estimation")
        if data is not None:
            np.save(GOOD_NPY, data)

    try:
        fast_data = np.load(FAST_NPY)   # cols: x,y,z,yaw,v,gas,brake
        good_data = np.load(GOOD_NPY)
    except FileNotFoundError:
        print("ERROR: trajectory .npy files missing.")
        return

    good_tree = KDTree(good_data[:, :2])

    with open("yas_marina_bnd.json") as f:
        track_boundaries = json.load(f).get("boundaries", {})

    # -----------------------------------------------------------------------
    # Camera frames — from fast_laps
    # -----------------------------------------------------------------------
    camera_frames = rip_camera_frames(
        "hackathon_fast_laps.mcap",
        "/constructor0/sensor/camera_fl/compressed_image",
    )
    if not camera_frames:
        print("WARNING: no camera frames found in fast_laps MCAP.")

    # -----------------------------------------------------------------------
    # Camera calibration
    # -----------------------------------------------------------------------
    calib = load_calibration("intrinsics/camera_fl_info.yaml")
    K = np.array(calib["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    dist = np.array(calib["distortion_coefficients"]["data"], dtype=np.float64)

    # -----------------------------------------------------------------------
    # Server
    # -----------------------------------------------------------------------
    foxglove.start_server(name="RacingCoach", host="0.0.0.0", port=8765)
    camera_chan      = CompressedImageChannel("/camera")
    camera_info_chan = CameraCalibrationChannel("/camera_info")
    scene_chan       = SceneUpdateChannel("/scene")

    print("Server live at ws://localhost:8765")

    num_state  = len(fast_data)
    num_frames = len(camera_frames)

    i = 0
    try:
        while True:
            now_ns = time.time_ns()
            ts = make_timestamp(now_ns)

            # Current car pose from fast_lap state estimation
            car_x, car_y, car_z, car_yaw = (
                fast_data[i, 0], fast_data[i, 1], fast_data[i, 2], fast_data[i, 3],
            )

            # ---------------------------------------------------------------
            # Camera image — annotated with good_lap projection
            # ---------------------------------------------------------------
            frame_idx = int(i / num_state * num_frames) % num_frames if num_frames else 0

            if camera_frames:
                raw_jpeg = camera_frames[frame_idx]
                frame = cv2.imdecode(np.frombuffer(raw_jpeg, np.uint8), cv2.IMREAD_COLOR)

                # Find good_lap points within 100 m ahead of car
                dists, idxs = good_tree.query(
                    [car_x, car_y], k=min(200, len(good_data))
                )
                nearby = good_data[idxs[dists < 100]]

                # Keep only points that are roughly in front (dot product with heading)
                fwd = np.array([np.cos(car_yaw), np.sin(car_yaw)])
                rel = nearby[:, :2] - np.array([car_x, car_y])
                ahead = rel @ fwd > 0
                nearby = nearby[ahead]

                pts3d = np.column_stack([
                    nearby[:, 0], nearby[:, 1],
                    np.full(len(nearby), car_z),  # project at ground level
                ])

                frame = project_trajectory_onto_frame(
                    frame, pts3d, car_x, car_y, car_z, car_yaw, K, dist,
                    color=(0, 255, 80), thickness=3,
                )

                _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                img_bytes = buf.tobytes()
            else:
                noise = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
                cv2.putText(noise, "NO CAMERA", (60, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                _, buf = cv2.imencode(".jpg", noise)
                img_bytes = buf.tobytes()

            camera_chan.log(
                CompressedImage(timestamp=ts, frame_id="camera_fl", data=img_bytes, format="jpeg"),
                log_time=now_ns,
            )

            # ---------------------------------------------------------------
            # Camera calibration
            # ---------------------------------------------------------------
            camera_info_chan.log(
                CameraCalibration(
                    timestamp=ts,
                    frame_id=calib["camera_name"],
                    width=calib["image_width"],
                    height=calib["image_height"],
                    distortion_model=calib["distortion_model"],
                    D=calib["distortion_coefficients"]["data"],
                    K=calib["camera_matrix"]["data"],
                    R=calib["rectification_matrix"]["data"],
                    P=calib["projection_matrix"]["data"],
                ),
                log_time=now_ns,
            )

            # ---------------------------------------------------------------
            # 3D Scene
            # ---------------------------------------------------------------
            entities = [
                SceneEntity(
                    id="fast_car", frame_id="map", timestamp=ts,
                    cubes=[CubePrimitive(
                        pose=Pose(
                            position=Vector3(x=car_x, y=car_y, z=car_z + 0.5),
                            orientation=Quaternion(w=np.cos(car_yaw/2), x=0, y=0, z=np.sin(car_yaw/2)),
                        ),
                        size=Vector3(x=4, y=2, z=1),
                        color=Color(r=1, g=0.6, b=0, a=1),
                    )],
                ),
            ]

            # Good lap line
            entities.append(SceneEntity(
                id="good_lap_line", frame_id="map",
                lines=[LinePrimitive(
                    type=LinePrimitiveLineType.LineStrip,
                    thickness=0.4,
                    color=Color(r=0, g=1, b=0.3, a=0.8),
                    points=[Point3(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in good_data[::5]],
                )],
            ))

            for name, pts in track_boundaries.items():
                entities.append(SceneEntity(
                    id=f"line_{name}", frame_id="map",
                    lines=[LinePrimitive(
                        type=LinePrimitiveLineType.LineStrip,
                        thickness=0.2,
                        color=Color(r=0.6, g=0.6, b=0.6, a=1),
                        points=[Point3(x=float(p[0]), y=float(p[1]), z=0) for p in pts],
                    )],
                ))

            scene_chan.log(SceneUpdate(entities=entities), log_time=now_ns)

            i = (i + 1) % num_state
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("Stopping...")


if __name__ == "__main__":
    main()
