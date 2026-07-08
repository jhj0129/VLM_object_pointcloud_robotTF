#!/usr/bin/env python3
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo


class DepthCenterProbe(Node):
    def __init__(self):
        super().__init__("depth_center_probe")

        self.declare_parameter("color_topic", "/test_rs/color/image_raw")
        self.declare_parameter("depth_topic", "/test_rs/aligned_depth_to_color/image_raw")
        self.declare_parameter("info_topic", "/test_rs/color/camera_info")
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("roi_half", 8)

        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.roi_half = int(self.get_parameter("roi_half").value)

        self.color = None
        self.depth = None
        self.info = None

        # RealSense/image pipeline topics often publish as Best Effort.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        info_topic = self.get_parameter("info_topic").value

        self.get_logger().info(f"subscribe color: {color_topic}")
        self.get_logger().info(f"subscribe depth: {depth_topic}")
        self.get_logger().info(f"subscribe info : {info_topic}")
        self.get_logger().info(f"depth_scale={self.depth_scale}, roi_half={self.roi_half}")

        self.create_subscription(Image, color_topic, self.color_cb, sensor_qos)
        self.create_subscription(Image, depth_topic, self.depth_cb, sensor_qos)
        self.create_subscription(CameraInfo, info_topic, self.info_cb, sensor_qos)

        self.timer = self.create_timer(0.3, self.tick)

    def image_to_np(self, msg):
        if msg.encoding in ("rgb8", "bgr8"):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == "bgr8":
                arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
            return arr.copy()

        if msg.encoding == "rgba8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
            arr = cv2.cvtColor(arr, cv2.COLOR_RGBA2RGB)
            return arr.copy()

        if msg.encoding == "bgra8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
            arr = cv2.cvtColor(arr, cv2.COLOR_BGRA2RGB)
            return arr.copy()

        self.get_logger().warn(f"unsupported color encoding: {msg.encoding}")
        return None

    def color_cb(self, msg):
        arr = self.image_to_np(msg)
        if arr is not None:
            self.color = arr

    def depth_cb(self, msg):
        if msg.encoding == "16UC1":
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            self.depth = arr.copy()
        elif msg.encoding == "32FC1":
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            # Convert meters to raw-like millimeters for unified handling.
            self.depth = (arr / self.depth_scale).astype(np.uint16)
        else:
            self.get_logger().warn(f"unexpected depth encoding: {msg.encoding}")

    def info_cb(self, msg):
        self.info = msg

    def tick(self):
        if self.color is None or self.depth is None or self.info is None:
            missing = []
            if self.color is None:
                missing.append("color")
            if self.depth is None:
                missing.append("depth")
            if self.info is None:
                missing.append("camera_info")
            self.get_logger().info(f"waiting for: {', '.join(missing)}")
            return

        h, w = self.depth.shape
        u = w // 2
        v = h // 2
        r = self.roi_half

        y0, y1 = max(0, v-r), min(h, v+r+1)
        x0, x1 = max(0, u-r), min(w, u+r+1)

        roi = self.depth[y0:y1, x0:x1]
        valid = roi[roi > 0]

        img = self.color.copy()
        cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 2)

        if valid.size > 0:
            med_raw = float(np.median(valid))
            mean_raw = float(np.mean(valid))
            min_raw = float(np.min(valid))
            max_raw = float(np.max(valid))

            med_m = med_raw * self.depth_scale
            mean_m = mean_raw * self.depth_scale
            min_m = min_raw * self.depth_scale
            max_m = max_raw * self.depth_scale

            fx = self.info.k[0]
            fy = self.info.k[4]
            cx = self.info.k[2]
            cy = self.info.k[5]

            x = (u - cx) * med_m / fx
            y = (v - cy) * med_m / fy

            txt = (
                f"center median={med_m:.3f}m "
                f"mean={mean_m:.3f}m "
                f"range=[{min_m:.3f},{max_m:.3f}]m "
                f"xyz=({x:.3f},{y:.3f},{med_m:.3f}) "
                f"valid={valid.size}/{roi.size}"
            )
            self.get_logger().info(txt)
            cv2.putText(img, txt[:95], (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
        else:
            txt = "center depth: NO VALID PIXELS"
            self.get_logger().warn(txt)
            cv2.putText(img, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,0,0), 2)

        cv2.imshow("depth_center_probe", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = DepthCenterProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
