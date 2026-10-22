from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cra_share = Path(
        get_package_share_directory("cra")
    )

    model_path = (
        cra_share
        / "models/robot_model/cognitive_robotics_arm.xml"
    )

    config_path = cra_share / "config/robot.yaml"

    workspace_path = LaunchConfiguration(
        "workspace_path"
    )

    graph_path = LaunchConfiguration(
        "graph_path"
    )

    goal_index = LaunchConfiguration(
        "goal_index"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "workspace_path",
            description="Absolute path to raw_workspace.npz",
        ),
        DeclareLaunchArgument(
            "graph_path",
            description="Absolute path to workspace_graph.npz",
        ),
        DeclareLaunchArgument(
            "goal_index",
            default_value="100",
        ),
        Node(
            package="planning",
            executable="planning_node",
            name="planning_node",
            output="screen",
            parameters=[{
                "workspace_path": workspace_path,
                "graph_path": graph_path,
                "model_path": str(model_path),
                "config_path": str(config_path),
                "goal_index": ParameterValue(
                    goal_index,
                    value_type=int,
                ),
                "joint_path_speed": 0.25,
                "feedback_timeout": 1.0,
                "obstacle_timeout": 1.0,
            }],
        ),
    ])