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

    raise RuntimeError("required config file not found")


def load_urdf_text(path):
    path = Path(path)

    if path.suffix == ".xacro" or path.name.endswith(".urdf.xacro"):
        result = subprocess.run(
            ["xacro", str(path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True,
        )
        return result.stdout

    return path.read_text()


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
            executable="moveit_grasp_candidate_plan_cpp_node",
            name="moveit_grasp_candidate_plan_cpp_node",
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

                    "input_point_topic": "/vlm_robot_tf/grasp_point",

                    "selected_grasp_topic": "/vlm_moveit/selected_grasp_pose",
                    "selected_pregrasp_topic": "/vlm_moveit/selected_pregrasp_pose",
                    "marker_topic": "/vlm_moveit/grasp_candidate_markers",
                    "display_topic": "/display_planned_path",

                    "planning_time": 5.0,
                    "planning_attempts": 10,
                    "goal_position_tolerance": 0.025,
                    "goal_orientation_tolerance": 0.60,
                    "velocity_scale": 0.15,
                    "acceleration_scale": 0.15,

                    "ik_timeout": 0.03,
                    "ik_attempts": 4,

                    "eef_step": 0.005,
                    "jump_threshold": 0.0,
                    "min_cartesian_fraction": 0.70,
                    "allow_regular_grasp_fallback": True,

                    "max_candidates_to_plan": 200,
                    "plan_once": True,
                    "avoid_collisions": True,
                    "require_current_state": True,

                    "low_z_threshold": 0.12,
                    "mid_z_threshold": 0.22,

                    "side_base_qx": 1.000,
                    "side_base_qy": 0.000,
                    "side_base_qz": -0.004,
                    "side_base_qw": -0.002,

                    "gripper_top_local_x": 0.0,
                    "gripper_top_local_y": 0.0,
                    "gripper_top_local_z": -1.0,

                    "joint4_weight": 3.0,
                    "joint5_weight": 3.0,
                    "joint6_weight": 10.0,
                    "other_joint_weight": 1.0,

                    "motion_score_weight": 1000.0,
                    "top_score_weight": 10.0,
                    "regular_fallback_penalty": 5.0,
                    "cartesian_fraction_penalty": 2.0,
                },
            ],
        )
    ])
