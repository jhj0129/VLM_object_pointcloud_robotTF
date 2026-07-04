from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input_point_topic", default_value="/vlm_robot_tf/grasp_point"),
        DeclareLaunchArgument("target_pose_topic", default_value="/vlm_moveit/grasp_target_pose"),
        DeclareLaunchArgument("pregrasp_pose_topic", default_value="/vlm_moveit/pregrasp_pose"),
        DeclareLaunchArgument("marker_topic", default_value="/vlm_moveit/target_markers"),
        DeclareLaunchArgument("target_frame", default_value="ARM_BASE_LINK"),

        DeclareLaunchArgument("target_offset_x", default_value="0.0"),
        DeclareLaunchArgument("target_offset_y", default_value="0.0"),
        DeclareLaunchArgument("target_offset_z", default_value="0.08"),

        DeclareLaunchArgument("pregrasp_extra_z", default_value="0.10"),

        DeclareLaunchArgument("target_qx", default_value="0.0"),
        DeclareLaunchArgument("target_qy", default_value="0.0"),
        DeclareLaunchArgument("target_qz", default_value="0.0"),
        DeclareLaunchArgument("target_qw", default_value="1.0"),

        DeclareLaunchArgument("target_marker_scale", default_value="0.025"),
        DeclareLaunchArgument("pregrasp_marker_scale", default_value="0.020"),

        Node(
            package="vlm_object_pointcloud_robot_tf",
            executable="robot_target_pose_cpp_node",
            name="robot_target_pose_cpp_node",
            output="screen",
            parameters=[
                {
                    "input_point_topic": LaunchConfiguration("input_point_topic"),

                    "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                    "pregrasp_pose_topic": LaunchConfiguration("pregrasp_pose_topic"),
                    "marker_topic": LaunchConfiguration("marker_topic"),

                    "target_frame": LaunchConfiguration("target_frame"),

                    "target_offset_x": ParameterValue(LaunchConfiguration("target_offset_x"), value_type=float),
                    "target_offset_y": ParameterValue(LaunchConfiguration("target_offset_y"), value_type=float),
                    "target_offset_z": ParameterValue(LaunchConfiguration("target_offset_z"), value_type=float),

                    "pregrasp_extra_z": ParameterValue(LaunchConfiguration("pregrasp_extra_z"), value_type=float),

                    "target_qx": ParameterValue(LaunchConfiguration("target_qx"), value_type=float),
                    "target_qy": ParameterValue(LaunchConfiguration("target_qy"), value_type=float),
                    "target_qz": ParameterValue(LaunchConfiguration("target_qz"), value_type=float),
                    "target_qw": ParameterValue(LaunchConfiguration("target_qw"), value_type=float),

                    "target_marker_scale": ParameterValue(LaunchConfiguration("target_marker_scale"), value_type=float),
                    "pregrasp_marker_scale": ParameterValue(LaunchConfiguration("pregrasp_marker_scale"), value_type=float),
                }
            ],
        )
    ])
