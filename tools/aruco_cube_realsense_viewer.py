#!/usr/bin/env python3
import math
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, String


def ros_image_to_bgr(msg: Image):
    h = msg.height
    w = msg.width
    enc = msg.encoding.lower()

    data = np.frombuffer(msg.data, dtype=np.uint8)

    if enc in ["bgr8"]:
        img = data.reshape((h, w, 3))
        return img.copy()

    if enc in ["rgb8"]:
        img = data.reshape((h, w, 3))
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    if enc in ["bgra8"]:
        img = data.reshape((h, w, 4))
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    if enc in ["rgba8"]:
        img = data.reshape((h, w, 4))
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

    if enc in ["mono8", "8uc1"]:
        img = data.reshape((h, w))
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")



def estimate_marker_poses_from_corners(corners, marker_length_m, camera_matrix, dist_coeffs):
    half = float(marker_length_m) * 0.5

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


class ArucoCubeViewer(Node):
    def __init__(self):
        super().__init__("aruco_cube_realsense_viewer")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("marker_length_m", 0.070)
        self.declare_parameter("valid_ids", [0, 1, 2, 3, 4, 5])
        self.declare_parameter("show_window", True)
        self.declare_parameter("window_name", "RealSense ArUco Cube Viewer")

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.marker_length_m = float(self.get_parameter("marker_length_m").value)
        self.valid_ids = set(int(x) for x in self.get_parameter("valid_ids").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.window_name = self.get_parameter("window_name").value

        if not hasattr(cv2, "aruco"):
            raise RuntimeError("cv2.aruco is missing. Install opencv-contrib or python3-opencv.")

        self.camera_matrix = None
        self.dist_coeffs = None
        self.last_log_time = 0.0
        self.frame_count = 0

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

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            qos,
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.on_camera_info,
            qos,
        )

        self.visible_pub = self.create_publisher(Bool, "/aruco_cube/visible", 10)
        self.ids_pub = self.create_publisher(String, "/aruco_cube/visible_ids", 10)

        self.get_logger().info("Aruco cube viewer started without cv_bridge")
        self.get_logger().info(f"image_topic: {self.image_topic}")
        self.get_logger().info(f"camera_info_topic: {self.camera_info_topic}")
        self.get_logger().info("dictionary: DICT_4X4_50")
        self.get_logger().info(f"valid ids: {sorted(self.valid_ids)}")
        self.get_logger().info(f"marker_length_m: {self.marker_length_m:.4f}")
        self.get_logger().info("Press q or ESC in the image window to quit")

    def on_camera_info(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def detect_markers(self, gray):
        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                self.dictionary,
                parameters=self.parameters,
            )
        return corners, ids, rejected

    def draw_pose_axes(self, image, corners, ids):
        if self.camera_matrix is None or self.dist_coeffs is None:
            return []

        detected_poses = []

        try:
            rvecs, tvecs = estimate_marker_poses_from_corners(
                corners,
                self.marker_length_m,
                self.camera_matrix,
                self.dist_coeffs,
            )
        except Exception as e:
            self.get_logger().warn(f"pose estimation failed: {e}")
            return []

        for i, marker_id in enumerate(ids.flatten()):
            marker_id = int(marker_id)

            if marker_id not in self.valid_ids:
                continue

            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

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

            detected_poses.append((marker_id, tvec, rvec))

        return detected_poses

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

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)

            for marker_id in ids.flatten():
                marker_id = int(marker_id)
                if marker_id in self.valid_ids:
                    visible_ids.append(marker_id)

            poses = self.draw_pose_axes(image, corners, ids)
        else:
            poses = []

        visible_ids = sorted(set(visible_ids))
        visible = len(visible_ids) > 0

        self.visible_pub.publish(Bool(data=visible))
        self.ids_pub.publish(String(data=",".join(str(x) for x in visible_ids)))

        h, w = image.shape[:2]

        if visible:
            status = f"VISIBLE ArUco IDs: {visible_ids}"
            status_color = (0, 255, 0)
        else:
            status = "NO ArUco ID 0~5 visible"
            status_color = (0, 0, 255)

        cv2.rectangle(image, (0, 0), (w, 92), (0, 0, 0), -1)

        cv2.putText(
            image,
            status,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            status_color,
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            image,
            f"DICT_4X4_50 | marker={self.marker_length_m*1000:.1f} mm | encoding={msg.encoding} | frame={self.frame_count}",
            (20, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        if self.camera_matrix is None:
            cv2.putText(
                image,
                "Waiting for camera_info...",
                (20, 88),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )

        y0 = 125
        for idx, (marker_id, tvec, _rvec) in enumerate(poses[:6]):
            x, y, z = float(tvec[0]), float(tvec[1]), float(tvec[2])
            dist = math.sqrt(x*x + y*y + z*z)

            text = (
                f"ID {marker_id}: "
                f"camera xyz=({x:.3f}, {y:.3f}, {z:.3f}) m, "
                f"dist={dist:.3f} m"
            )

            cv2.putText(
                image,
                text,
                (20, y0 + idx * 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        now = time.time()
        if now - self.last_log_time > 1.0:
            if visible:
                self.get_logger().info(f"VISIBLE ids={visible_ids}")
            else:
                self.get_logger().warn("NO marker visible")
            self.last_log_time = now

        if self.show_window:
            cv2.imshow(self.window_name, image)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == 27:
                self.get_logger().info("quit requested")
                rclpy.shutdown()


def main():
    rclpy.init()
    node = ArucoCubeViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    main()
