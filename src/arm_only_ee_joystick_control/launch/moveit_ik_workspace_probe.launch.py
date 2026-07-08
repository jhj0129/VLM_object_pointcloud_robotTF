from pathlib import Path
import subprocess
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml_file(path):
    path = Path(path)
    if not path.exists():
        return {}
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    return data if data is not None else {}


def find_first(config_dir, candidates, patterns):
    for name in candidates:
        p = config_dir / name
        if p.exists():
            return p

    for pattern in patterns:
        matches = sorted(config_dir.glob(pattern))
        if matches:
            return matches[0]

    raise RuntimeError(f"Could not find required config file in {config_dir}")


def load_urdf_text(urdf_path):
    urdf_path = Path(urdf_path)

    if urdf_path.suffix == ".xacro" or urdf_path.name.endswith(".urdf.xacro"):
        result = subprocess.run(
            ["xacro", str(urdf_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            check=True,
        )
        return result.stdout

    return urdf_path.read_text()


def generate_launch_description():
    moveit_config_dir = Path(
        get_package_share_directory("arm_only_moveit_config")
    ) / "config"

    urdf_path = find_first(
        moveit_config_dir,
        [
            "arm_only_gripper.urdf",
            "arm_only_gripper.urdf.xacro",
            "arm_only_clean.urdf",
            "arm_only_clean.urdf.xacro",
            "robot.urdf",
            "robot.urdf.xacro",
        ],
        ["*.urdf", "*.urdf.xacro", "*.xacro"],
    )

    srdf_path = find_first(
        moveit_config_dir,
        [
            "arm_only_gripper.srdf",
            "arm_only_clean.srdf",
            "robot.srdf",
        ],
        ["*.srdf"],
    )

    kinematics_path = moveit_config_dir / "kinematics.yaml"
    joint_limits_path = moveit_config_dir / "joint_limits.yaml"

    return LaunchDescription([
        Node(
            package="vlm_object_pointcloud_robot_tf",
            executable="moveit_ik_workspace_probe_cpp_node",
            name="moveit_ik_workspace_probe_cpp_node",
            output="screen",
            parameters=[
                {
                    "robot_description": ParameterValue(
                        load_urdf_text(urdf_path),
                        value_type=str,
                    )
                },
                {
                    "robot_description_semantic": ParameterValue(
                        srdf_path.read_text(),
                        value_type=str,
                    )
                },
                {
                    "robot_description_kinematics": load_yaml_file(kinematics_path),
                    "robot_description_planning": load_yaml_file(joint_limits_path),

                    "planning_group": "arm",
                    "tip_link": "gripper_tcp",
                    "frame_id": "ARM_BASE_LINK",

                    "x_min": 0.10,
                    "x_max": 0.75,
                    "y_min": -0.35,
                    "y_max": 0.35,
                    "z_min": -0.10,
                    "z_max": 0.55,
                    "step": 0.05,

                    "target_qx": 1.000,
                    "target_qy": 0.000,
                    "target_qz": -0.004,
                    "target_qw": -0.002,

                    "ik_timeout": 0.01,
                    "attempts": 8,
                    "pos_tol": 0.005,
                    "rot_tol": 0.10,

                    "marker_topic": "/vlm_debug/ik_workspace_markers",
                    "csv_path": "/tmp/vlm_side_grasp_ik_workspace.csv",
                },
            ],
        )
    ])
