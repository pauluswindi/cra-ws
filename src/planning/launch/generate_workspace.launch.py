from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    workspace_dir = Path.home() / "living-lab" / "cra_ws"

    default_output = (
        workspace_dir
        / "src"
        / "planning"
        / "data"
        / "raw_workspace.npz"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "output",
            default_value=str(default_output),
            description="Output workspace NPZ file",
        ),
        ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "planning",
                "generate_workspace",
                "--output",
                LaunchConfiguration("output"),
            ],
            output="screen",
        ),
    ])