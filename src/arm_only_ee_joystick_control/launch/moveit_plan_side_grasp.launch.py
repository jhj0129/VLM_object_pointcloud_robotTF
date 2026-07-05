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
            "arm_only.urdf",
            "arm_only.urdf.xacro",
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
            "arm_only.srdf",
            "robot.srdf",
        ],
        ["*.srdf"],
    )

    kinematics_path = moveit_config_dir / "kinematics.yaml"
    joint_limits_path = moveit_config_dir / "joint_limits.yaml"

    robot_description = {
        "robot_description": ParameterValue(
            load_urdf_text(urdf_path),
            value_type=str,
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            srdf_path.read_text(),
            value_type=str,
        )
    }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml_file(kinematics_path)
    }

    robot_description_planning = {
        "robot_description_planning": load_yaml_file(joint_limits_path)
    }

    print("[moveit_plan_side_grasp] URDF:", urdf_path)
    print("[moveit_plan_side_grasp] SRDF:", srdf_path)
    print("[moveit_plan_side_grasp] kinematics:", kinematics_path)
    print("[moveit_plan_side_grasp] joint_limits:", joint_limits_path)

    return LaunchDescription([
        Node(
            package="vlm_object_pointcloud_robot_tf",
            executable="moveit_plan_side_grasp_cpp_node",
            name="moveit_plan_side_grasp_cpp_node",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                robot_description_planning,
                {
                    "pregrasp_pose_topic": "/vlm_moveit/pregrasp_pose",
                    "grasp_pose_topic": "/vlm_moveit/grasp_target_pose",

                    "planning_group": "arm",
                    "end_effector_link": "gripper_tcp",

                    "plan_once": True,

                    "planning_time": 8.0,
                    "planning_attempts": 20,

                    "velocity_scaling": 0.10,
                    "accel_scaling": 0.10,

                    "cartesian_eef_step": 0.005,
                    "cartesian_jump_threshold": 0.0,
                    "cartesian_min_fraction": 0.95,
                    "avoid_collisions": True,

                    "display_trajectory_topic": "/display_planned_path",
                },
            ],
        )
    ])
