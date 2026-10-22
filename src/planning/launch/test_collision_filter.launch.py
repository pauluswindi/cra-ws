from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    workspace_dir = (
        Path.home() / "living-lab" / "cra_ws"
    )

    graph_path = (
        workspace_dir
        / "src/planning/data/workspace_graph.npz"
    )

    model_path = (
        workspace_dir
        / "src/cra/models/robot_model/cognitive_robotics_arm.xml"
    )

    config_path = (
        workspace_dir
        / "src/cra/config/robot.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "graph",
            default_value=str(graph_path),
        ),
        DeclareLaunchArgument(
            "model",
            default_value=str(model_path),
        ),
        DeclareLaunchArgument(
            "config",
            default_value=str(config_path),
        ),
        ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "planning",
                "collision_filter",
                "--graph",
                LaunchConfiguration("graph"),
                "--model",
                LaunchConfiguration("model"),
                "--config",
                LaunchConfiguration("config"),
            ],
            output="screen",
        ),
    ])