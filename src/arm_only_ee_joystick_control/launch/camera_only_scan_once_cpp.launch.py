from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="arm_only_ee_joystick_control",
            executable="camera_only_scan_once_cpp_node",
            name="camera_only_scan_once_cpp_node",
            output="screen",
            parameters=[
                {
                    "rgb_topic": "/test_rs/color/image_raw",
                    "depth_topic": "/test_rs/depth/image_raw",
                    "info_topic": "/test_rs/color/camera_info",

                    # Stage 1: camera frame only. No robot TF.
                    "frame_id": "camera_color_optical_frame",

                    # Baseline color target.
                    # Later: Qwen bbox + SAM mask.
                    "target_color": "blue",

                    # 20 frames at 15Hz = about 1.33 sec scan.
                    "capture_frames": 20,

                    "depth_scale": 0.001,
                    "raw_depth_min": 0.10,
                    "raw_depth_max": 1.50,

                    "min_mask_pixels": 80,
                    "min_valid_depth_pixels": 50,
                    "max_cloud_points": 5000,

                    # RViz marker size.
                    # green sphere = grasp point, orange cube = push point
                    "grasp_marker_scale": 0.015,
                    "push_marker_scale": 0.012,

                    # Keep result alive for RViz.
                    "publish_period": 1.0,

                    "cloud_topic": "/vlm_camera_scan/object_cloud",
                    "grasp_point_topic": "/vlm_camera_scan/grasp_point",
                    "push_point_topic": "/vlm_camera_scan/push_point",
                    "marker_topic": "/vlm_camera_scan/object_markers",
                    "annotated_topic": "/vlm_camera_scan/annotated_image",
                    "mask_topic": "/vlm_camera_scan/mask_image",
                }
            ],
        )
    ])
