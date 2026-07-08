#!/usr/bin/env python3
import csv
import math
import os
import time
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, String

from tf2_ros import Buffer, TransformListener


def ros_image_to_bgr(msg: Image):
    h = msg.height
    w = msg.width
    enc = msg.encoding.lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if enc == "bgr8":
        row = msg.step
        img = data.reshape((h, row))[:, :w * 3].reshape((h, w, 3))
        return img.copy()

    if enc == "rgb8":
        row = msg.step
        img = data.reshape((h, row))[:, :w * 3].reshape((h, w, 3))
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    if enc == "bgra8":
        row = msg.step
        img = data.reshape((h, row))[:, :w * 4].reshape((h, w, 4))
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    if enc == "rgba8":
        row = msg.step
        img = data.reshape((h, row))[:, :w * 4].reshape((h, w, 4))
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

    if enc in ["mono8", "8uc1"]:
        row = msg.step
        img = data.reshape((h, row))[:, :w]
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")



def estimate_marker_poses_from_corners(corners, marker_length_m, camera_matrix, dist_coeffs):
    half = float(marker_length_m) * 0.5

    # Corner order from OpenCV ArUco:
    # top-left, top-right, bottom-right, bottom-left
    obj_pts = np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float32)

    rvecs = []
    tvecs = []

    flags = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)

    for c in corners:
        img_pts = np.asarray(c, dtype=np.float32).reshape(4, 2)

        ok, rvec, tvec = cv2.solvePnP(
            obj_pts,
            img_pts,
            camera_matrix,
            dist_coeffs,
            flags=flags,
        )

        if not ok and flags != cv2.SOLVEPNP_ITERATIVE:
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts,
                img_pts,
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

        if not ok:
            raise RuntimeError("cv2.solvePnP failed for one marker")

        rvecs.append(rvec.reshape(3))
        tvecs.append(tvec.reshape(3))

    return np.asarray(rvecs, dtype=np.float64).reshape(-1, 1, 3), np.asarray(tvecs, dtype=np.float64).reshape(-1, 1, 3)


def quat_xyzw_to_R(qx, qy, qz, qw):
    q = np.array([qx, qy, qz, qw], dtype=float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q

    return np.array([
        [1 - 2 * (y*y + z*z),     2 * (x*y - z*w),       2 * (x*z + y*w)],
        [2 * (x*y + z*w),         1 - 2 * (x*x + z*z),   2 * (y*z - x*w)],
        [2 * (x*z - y*w),         2 * (y*z + x*w),       1 - 2 * (x*x + y*y)],
    ], dtype=float)


def tf_to_R_t(tf_msg):
    tr = tf_msg.transform.translation
    qr = tf_msg.transform.rotation
    R = quat_xyzw_to_R(qr.x, qr.y, qr.z, qr.w)
    t = np.array([tr.x, tr.y, tr.z], dtype=float)
    return R, t


def matrix_to_quat_xyzw(R):
    tr = np.trace(R)

    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    q = np.array([qx, qy, qz, qw], dtype=float)
    q = q / np.linalg.norm(q)
    return q


def matrix_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    cp = math.cos(pitch)

    if abs(cp) > 1e-8:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])

    return roll, pitch, yaw


def solve_rigid_transform(camera_points, base_points):
    C = np.asarray(camera_points, dtype=float)
    B = np.asarray(base_points, dtype=float)

    C_mean = C.mean(axis=0)
    B_mean = B.mean(axis=0)

    C0 = C - C_mean
    B0 = B - B_mean

    H = C0.T @ B0
    U, S, Vt = np.linalg.svd(H)

    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1.0
        R = Vt.T @ U.T

    t = B_mean - R @ C_mean

    pred = (R @ C.T).T + t
    err = np.linalg.norm(pred - B, axis=1)

    return R, t, err


class ArucoCubeTfCalibrator(Node):
    def __init__(self):
        super().__init__("aruco_cube_tf_calibrator")

        self.declare_parameter("image_topic", "/test_rs/color/image_raw")
        self.declare_parameter("camera_info_topic", "/test_rs/color/camera_info")

        self.declare_parameter("base_frame", "ARM_BASE_LINK")
        self.declare_parameter("tcp_frame", "gripper_tcp")

        self.declare_parameter("marker_length_m", 0.070)
        self.declare_parameter("cube_side_m", 0.095)

        self.declare_parameter("marker_z_sign", 1.0)
        self.declare_parameter("display_scale", 1.6)
        self.declare_parameter("valid_ids", [0, 1, 2, 3, 4, 5])

        self.declare_parameter("tcp_to_cube_xyz", [0.0, 0.0, 0.0])

        self.declare_parameter("csv_path", "/tmp/aruco_cube_tf_samples.csv")
        self.declare_parameter("result_path", "/tmp/aruco_cube_tf_result.txt")

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value

        self.base_frame = self.get_parameter("base_frame").value
        self.tcp_frame = self.get_parameter("tcp_frame").value

        self.marker_length_m = float(self.get_parameter("marker_length_m").value)
        self.cube_side_m = float(self.get_parameter("cube_side_m").value)
        self.cube_half_m = self.cube_side_m * 0.5

        self.marker_z_sign = float(self.get_parameter("marker_z_sign").value)
        self.display_scale = float(self.get_parameter("display_scale").value)

        self.valid_ids = set(int(x) for x in self.get_parameter("valid_ids").value)

        tcp_to_cube = self.get_parameter("tcp_to_cube_xyz").value
        self.tcp_to_cube_xyz = np.array([float(tcp_to_cube[0]), float(tcp_to_cube[1]), float(tcp_to_cube[2])], dtype=float)

        self.csv_path = Path(self.get_parameter("csv_path").value)
        self.result_path = Path(self.get_parameter("result_path").value)

        if not hasattr(cv2, "aruco"):
            raise RuntimeError("cv2.aruco is missing. Install OpenCV aruco/contrib support.")

        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = "UNKNOWN_CAMERA_FRAME"

        self.latest_cube_center_camera = None
        self.latest_visible_ids = []
        self.latest_marker_rows = []
        self.latest_stamp_time = 0.0

        self.samples_camera = []
        self.samples_base = []
        self.samples_meta = []

        self.frame_count = 0
        self.last_log_time = 0.0

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        if hasattr(cv2.aruco, "DetectorParameters"):
            self.parameters = cv2.aruco.DetectorParameters()
        else:
            self.parameters = cv2.aruco.DetectorParameters_create()

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        else:
            self.detector = None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.image_sub = self.create_subscription(Image, self.image_topic, self.on_image, qos)
        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, qos)

        self.visible_pub = self.create_publisher(Bool, "/aruco_cube/visible", 10)
        self.ids_pub = self.create_publisher(String, "/aruco_cube/visible_ids", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.csv_file = self.csv_path.open("w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "sample",
            "camera_frame",
            "camera_x", "camera_y", "camera_z",
            "base_frame",
            "base_x", "base_y", "base_z",
            "visible_ids",
            "marker_count",
        ])
        self.csv_file.flush()

        self.create_timer(2.0, self.heartbeat)

        self.get_logger().info("Aruco cube TF calibrator started")
        self.get_logger().info(f"image_topic: {self.image_topic}")
        self.get_logger().info(f"camera_info_topic: {self.camera_info_topic}")
        self.get_logger().info(f"base_frame: {self.base_frame}")
        self.get_logger().info(f"tcp_frame: {self.tcp_frame}")
        self.get_logger().info(f"marker_length_m: {self.marker_length_m:.6f}")
        self.get_logger().info(f"cube_side_m: {self.cube_side_m:.6f}")
        self.get_logger().info(f"cube_half_m: {self.cube_half_m:.6f}")
        self.get_logger().info(f"marker_z_sign: {self.marker_z_sign}")
        self.get_logger().info(f"display_scale: {self.display_scale}")
        self.get_logger().info(f"tcp_to_cube_xyz: {self.tcp_to_cube_xyz.tolist()}")
        self.get_logger().info("Controls: press 's' to save sample, 'q' or ESC to quit")

    def heartbeat(self):
        self.get_logger().info(
            f"heartbeat: frames={self.frame_count}, "
            f"camera_info={'OK' if self.camera_matrix is not None else 'WAIT'}, "
            f"samples={len(self.samples_camera)}, "
            f"visible_ids={self.latest_visible_ids}"
        )

    def on_camera_info(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        self.camera_frame = msg.header.frame_id if msg.header.frame_id else "camera_color_optical_frame"

    def detect_markers(self, gray):
        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        return corners, ids, rejected

    def on_image(self, msg: Image):
        self.frame_count += 1

        try:
            image = ros_image_to_bgr(msg)
        except Exception as e:
            self.get_logger().error(f"image conversion failed: {e}")
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detect_markers(gray)

        visible_ids = []
        marker_rows = []
        cube_centers = []

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)

            if self.camera_matrix is not None and self.dist_coeffs is not None:
                try:
                    rvecs, tvecs = estimate_marker_poses_from_corners(
                        corners,
                        self.marker_length_m,
                        self.camera_matrix,
                        self.dist_coeffs,
                    )

                    for i, marker_id_raw in enumerate(ids.flatten()):
                        marker_id = int(marker_id_raw)

                        if marker_id not in self.valid_ids:
                            continue

                        rvec = rvecs[i][0]
                        tvec = tvecs[i][0].astype(float)

                        R_marker, _ = cv2.Rodrigues(rvec)

                        # OpenCV marker local +Z is usually the direction into the board/cube
                        # for a front-facing marker. If your result is clearly wrong,
                        # rerun with marker_z_sign:=-1.0.
                        p_marker_to_cube = np.array(
                            [0.0, 0.0, self.marker_z_sign * self.cube_half_m],
                            dtype=float,
                        )

                        p_cube_camera = tvec + R_marker @ p_marker_to_cube

                        cube_centers.append(p_cube_camera)
                        visible_ids.append(marker_id)

                        try:
                            cv2.drawFrameAxes(
                                image,
                                self.camera_matrix,
                                self.dist_coeffs,
                                rvec,
                                tvec,
                                self.marker_length_m * 0.5,
                            )
                        except Exception:
                            pass

                        marker_rows.append((marker_id, tvec, p_cube_camera))

                except Exception as e:
                    self.get_logger().warn(f"pose estimation failed: {e}")

        visible_ids = sorted(set(visible_ids))

        if len(cube_centers) > 0:
            self.latest_cube_center_camera = np.mean(np.asarray(cube_centers), axis=0)
            self.latest_stamp_time = time.time()
        else:
            self.latest_cube_center_camera = None

        self.latest_visible_ids = visible_ids
        self.latest_marker_rows = marker_rows

        visible = len(visible_ids) > 0
        self.visible_pub.publish(Bool(data=visible))
        self.ids_pub.publish(String(data=",".join(str(x) for x in visible_ids)))

        self.draw_overlay(image, msg.encoding)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):
            self.save_current_sample()

        if key == ord("q") or key == 27:
            self.get_logger().info("quit requested")
            self.solve_and_report(force=True)
            try:
                self.csv_file.close()
            except Exception:
                pass
            rclpy.shutdown()

    def draw_overlay(self, image, encoding):
        h, w = image.shape[:2]

        face_name = {
            0: "FRONT",
            1: "BACK",
            2: "LEFT",
            3: "RIGHT",
            4: "TOP",
            5: "BOTTOM",
        }

        visible_faces = [
            f"{face_name.get(i, 'ID')}({i})"
            for i in self.latest_visible_ids
        ]

        if self.latest_visible_ids:
            status = "VISIBLE: " + ", ".join(visible_faces)
            color = (0, 255, 0)
        else:
            status = "NO MARKER"
            color = (0, 0, 255)

        cv2.rectangle(image, (0, 0), (w, 82), (0, 0, 0), -1)

        cv2.putText(
            image,
            status,
            (18, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.95,
            color,
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            image,
            f"S=save | Q=quit | samples={len(self.samples_camera)} | frame={self.camera_frame}",
            (18, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        if self.latest_visible_ids:
            cv2.putText(
                image,
                "READY",
                (w - 150, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.1,
                (0, 255, 0),
                3,
                cv2.LINE_AA,
            )

        if self.camera_matrix is None:
            cv2.putText(
                image,
                "WAIT CAMERA_INFO",
                (18, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        display = image

        if self.display_scale != 1.0:
            display = cv2.resize(
                image,
                None,
                fx=self.display_scale,
                fy=self.display_scale,
                interpolation=cv2.INTER_LINEAR,
            )

        cv2.namedWindow("Aruco Cube TF Calibrator", cv2.WINDOW_NORMAL)
        cv2.imshow("Aruco Cube TF Calibrator", display)

    def save_current_sample(self):
        if self.latest_cube_center_camera is None:
            self.get_logger().warn("cannot save: no cube center visible")
            return

        if time.time() - self.latest_stamp_time > 1.0:
            self.get_logger().warn("cannot save: latest cube center is too old")
            return

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tcp_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.3),
            )
        except Exception as e:
            self.get_logger().error(f"cannot save: TF lookup failed {self.base_frame} <- {self.tcp_frame}: {e}")
            return

        R_base_tcp, t_base_tcp = tf_to_R_t(tf_msg)

        p_camera_cube = self.latest_cube_center_camera.copy()
        p_base_cube = t_base_tcp + R_base_tcp @ self.tcp_to_cube_xyz

        self.samples_camera.append(p_camera_cube)
        self.samples_base.append(p_base_cube)

        sample_idx = len(self.samples_camera)

        self.csv_writer.writerow([
            sample_idx,
            self.camera_frame,
            f"{p_camera_cube[0]:.9f}",
            f"{p_camera_cube[1]:.9f}",
            f"{p_camera_cube[2]:.9f}",
            self.base_frame,
            f"{p_base_cube[0]:.9f}",
            f"{p_base_cube[1]:.9f}",
            f"{p_base_cube[2]:.9f}",
            ",".join(str(x) for x in self.latest_visible_ids),
            len(self.latest_marker_rows),
        ])
        self.csv_file.flush()

        self.get_logger().info(
            f"SAVED sample {sample_idx}: "
            f"camera_cube=({p_camera_cube[0]:.4f},{p_camera_cube[1]:.4f},{p_camera_cube[2]:.4f}) "
            f"base_cube=({p_base_cube[0]:.4f},{p_base_cube[1]:.4f},{p_base_cube[2]:.4f}) "
            f"ids={self.latest_visible_ids}"
        )

        if sample_idx >= 6:
            self.solve_and_report(force=False)
        else:
            self.get_logger().info(f"Need at least 6 samples. Current: {sample_idx}")

    def solve_and_report(self, force=False):
        n = len(self.samples_camera)

        if n < 6:
            if force:
                self.get_logger().warn(f"not enough samples to solve. Need >= 6, current={n}")
            return

        R, t, err = solve_rigid_transform(self.samples_camera, self.samples_base)

        qx, qy, qz, qw = matrix_to_quat_xyzw(R)
        roll, pitch, yaw = matrix_to_rpy(R)

        mean_mm = float(np.mean(err) * 1000.0)
        median_mm = float(np.median(err) * 1000.0)
        max_mm = float(np.max(err) * 1000.0)

        text = []
        text.append("")
        text.append("===== ARUCO CUBE CAMERA-ROBOT TF RESULT =====")
        text.append(f"samples: {n}")
        text.append(f"camera_frame: {self.camera_frame}")
        text.append(f"base_frame: {self.base_frame}")
        text.append(f"tcp_frame: {self.tcp_frame}")
        text.append(f"mean_error_mm:   {mean_mm:.3f}")
        text.append(f"median_error_mm: {median_mm:.3f}")
        text.append(f"max_error_mm:    {max_mm:.3f}")
        text.append("")
        text.append(f"T_{self.base_frame}_{self.camera_frame}:")
        text.append(f"x:  {t[0]:.9f}")
        text.append(f"y:  {t[1]:.9f}")
        text.append(f"z:  {t[2]:.9f}")
        text.append(f"qx: {qx:.12f}")
        text.append(f"qy: {qy:.12f}")
        text.append(f"qz: {qz:.12f}")
        text.append(f"qw: {qw:.12f}")
        text.append("")
        text.append("RPY rad:")
        text.append(f"roll:  {roll:.12f}")
        text.append(f"pitch: {pitch:.12f}")
        text.append(f"yaw:   {yaw:.12f}")
        text.append("")
        text.append("Static TF command candidate:")
        text.append("ros2 run tf2_ros static_transform_publisher \\")
        text.append(f"  --x {t[0]:.9f} \\")
        text.append(f"  --y {t[1]:.9f} \\")
        text.append(f"  --z {t[2]:.9f} \\")
        text.append(f"  --qx {qx:.12f} \\")
        text.append(f"  --qy {qy:.12f} \\")
        text.append(f"  --qz {qz:.12f} \\")
        text.append(f"  --qw {qw:.12f} \\")
        text.append(f"  --frame-id {self.base_frame} \\")
        text.append(f"  --child-frame-id {self.camera_frame}")
        text.append("")
        text.append("Per-sample error mm:")
        for i, e in enumerate(err, 1):
            text.append(f"{i:02d}: {e * 1000.0:.3f}")

        result = "\n".join(text)

        print(result)

        self.result_path.write_text(result + "\n")
        self.get_logger().info(f"saved result: {self.result_path}")
        self.get_logger().info(f"saved samples: {self.csv_path}")


def main():
    rclpy.init()
    node = ArucoCubeTfCalibrator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.solve_and_report(force=True)
    finally:
        try:
            node.csv_file.close()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    main()
