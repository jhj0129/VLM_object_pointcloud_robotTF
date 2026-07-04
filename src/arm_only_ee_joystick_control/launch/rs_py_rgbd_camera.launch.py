from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="vlm_object_pointcloud_robot_tf",
            executable="rs_py_rgbd_camera_node",
            name="rs_py_rgbd_camera_node",
            output="screen",
            parameters=[
                {
                    "width": 424,
                    "height": 240,
                    "fps": 15,
                    "color_frame_id": "camera_color_optical_frame",
                    "depth_frame_id": "camera_color_optical_frame",
                    "color_topic": "/test_rs/color/image_raw",
                    "color_info_topic": "/test_rs/color/camera_info",
                    "depth_topic": "/test_rs/depth/image_raw",
                    "depth_info_topic": "/test_rs/depth/camera_info",
                    "aligned_depth_topic": "/test_rs/aligned_depth_to_color/image_raw",
                    "aligned_depth_info_topic": "/test_rs/aligned_depth_to_color/camera_info",
                }
            ],
        )
    ])
