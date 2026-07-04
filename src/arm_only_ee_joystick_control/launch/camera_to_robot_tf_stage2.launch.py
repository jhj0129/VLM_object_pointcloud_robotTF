from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")

    camera_qx = LaunchConfiguration("camera_qx")
    camera_qy = LaunchConfiguration("camera_qy")
    camera_qz = LaunchConfiguration("camera_qz")
    camera_qw = LaunchConfiguration("camera_qw")

    parent_frame = LaunchConfiguration("parent_frame")
    camera_frame = LaunchConfiguration("camera_frame")

    return LaunchDescription([
        DeclareLaunchArgument("parent_frame", default_value="ARM_BASE_LINK"),
        DeclareLaunchArgument("camera_frame", default_value="camera_color_optical_frame"),

        # Measured camera origin relative to ARM_BASE_LINK.
        # Unit: meter
        DeclareLaunchArgument("camera_x", default_value="-0.010"),
        DeclareLaunchArgument("camera_y", default_value="0.323"),
        DeclareLaunchArgument("camera_z", default_value="0.457"),

        # Temporary optical-frame rotation.
        DeclareLaunchArgument("camera_qx", default_value="0.5"),
        DeclareLaunchArgument("camera_qy", default_value="-0.5"),
        DeclareLaunchArgument("camera_qz", default_value="0.5"),
        DeclareLaunchArgument("camera_qw", default_value="-0.5"),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="stage2_static_tf_arm_base_to_camera",
            output="screen",
            arguments=[
                "--x", camera_x,
                "--y", camera_y,
                "--z", camera_z,
                "--qx", camera_qx,
                "--qy", camera_qy,
                "--qz", camera_qz,
                "--qw", camera_qw,
                "--frame-id", parent_frame,
                "--child-frame-id", camera_frame,
            ],
        ),

        Node(
            package="vlm_object_pointcloud_robot_tf",
            executable="camera_to_robot_tf_cpp_node",
            name="camera_to_robot_tf_cpp_node",
            output="screen",
            parameters=[
                {
                    "target_frame": "ARM_BASE_LINK",

                    "input_cloud_topic": "/vlm_camera_scan/object_cloud",
                    "input_grasp_point_topic": "/vlm_camera_scan/grasp_point",
                    "input_push_point_topic": "/vlm_camera_scan/push_point",

                    "output_cloud_topic": "/vlm_robot_tf/object_cloud",
                    "output_grasp_point_topic": "/vlm_robot_tf/grasp_point",
                    "output_push_point_topic": "/vlm_robot_tf/push_point",
                    "output_marker_topic": "/vlm_robot_tf/object_markers",

                    "grasp_marker_scale": 0.015,
                    "push_marker_scale": 0.012,
                }
            ],
        ),
    ])
