from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="vlm_object_pointcloud_robot_tf",
            executable="robot_target_pose_cpp_node",
            name="robot_target_pose_cpp_node",
            output="screen",
            parameters=[
                {
                    "input_point_topic": "/vlm_robot_tf/grasp_point",

                    "target_pose_topic": "/vlm_moveit/grasp_target_pose",
                    "pregrasp_pose_topic": "/vlm_moveit/pregrasp_pose",
                    "marker_topic": "/vlm_moveit/target_markers",

                    "target_frame": "ARM_BASE_LINK",

                    # object center -> EE target offset
                    "target_offset_x": 0.0,
                    "target_offset_y": 0.0,
                    "target_offset_z": 0.08,

                    # pregrasp = target + z
                    "pregrasp_extra_z": 0.10,

                    # Temporary orientation.
                    # This is only for visualization now.
                    # MoveIt planning orientation will be tuned in the next step.
                    "target_qx": 0.0,
                    "target_qy": 0.0,
                    "target_qz": 0.0,
                    "target_qw": 1.0,

                    "target_marker_scale": 0.025,
                    "pregrasp_marker_scale": 0.020,
                }
            ],
        )
    ])
