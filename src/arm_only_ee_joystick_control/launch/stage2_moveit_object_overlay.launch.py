from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    moveit_demo_launch = (
        Path(get_package_share_directory("arm_only_moveit_config"))
        / "launch"
        / "demo.launch.py"
    )

    stage2_tf_launch = (
        Path(get_package_share_directory("vlm_object_pointcloud_robot_tf"))
        / "launch"
        / "camera_to_robot_tf_stage2.launch.py"
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(moveit_demo_launch))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(stage2_tf_launch))
        ),
    ])
