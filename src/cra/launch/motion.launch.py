from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    cra_share = Path(get_package_share_directory("cra"))
    config_path = cra_share / "config" / "robot.yaml"

    with config_path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file) or {}

    robot = config["robot"]
    ik = config["kinematics"]["inverse"]
    jacobian = config["kinematics"]["jacobian"]

    motion_config = config["motion"]
    tracking = motion_config["tracking"]
    joint_tracking = motion_config.get("joint_tracking", {})

    trajectory = config["trajectory"]

    xml_path = (
        cra_share / robot["model"]["path"]
    ).resolve()

    if not xml_path.is_file():
        raise FileNotFoundError(
            f"Robot XML not found: {xml_path}"
        )

    parameters = {
        "robot_model_path": str(xml_path),
        "base_frame": robot["frames"]["base"],
        "end_effector_frame": robot["frames"]["end_effector"],

        "ik.max_iterations": int(ik["max_iterations"]),
        "ik.tolerance": float(ik["tolerance"]),
        "ik.damping": float(ik["damping"]),
        "ik.step_size": float(ik["step_size"]),
        "ik.max_joint_step": float(ik["max_joint_step"]),

        "jacobian.damping": float(jacobian["damping"]),

        "tracking_gain": float(tracking["gain"]),
        "max_cartesian_velocity": float(
            tracking["max_cartesian_velocity"]
        ),
        "max_joint_velocity": float(
            tracking["max_joint_velocity"]
        ),
        "target_tolerance": float(
            tracking["target_tolerance"]
        ),
        "feedback_timeout": float(
            tracking["feedback_timeout"]
        ),

        "control_mode": str(
            motion_config.get("control_mode", "cartesian")
        ),

        "target_source": str(
            motion_config.get("target_source", "legacy")
        ),

        "joint_tracking_gain": float(
            joint_tracking.get("gain", 2.0)
        ),
        "joint_goal_tolerance": float(
            joint_tracking.get("goal_tolerance", 0.02)
        ),
        "joint_start_tolerance": float(
            joint_tracking.get("start_tolerance", 0.10)
        ),
        "joint_max_segment_delta": float(
            joint_tracking.get("max_segment_delta", 0.6)
        ),

        "trajectory.enabled": bool(trajectory["enabled"]),
        "trajectory.speed": float(trajectory["speed"]),
        "trajectory.distance_step": float(
            trajectory["distance_step"]
        ),
        "trajectory.curve": str(trajectory["curve"]),
        "trajectory.replan_threshold": float(
            trajectory["replan_threshold"]
        ),
    }

    motion_node = ComposableNode(
        package="motion",
        plugin="motion::MotionComponent",
        name="motion_component",
        parameters=[parameters],
    )

    container = ComposableNodeContainer(
        name="motion_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[motion_node],
        output="screen",
    )

    return LaunchDescription([container])