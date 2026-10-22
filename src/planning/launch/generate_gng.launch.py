from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    data_dir = (
        Path.home()
        / "living-lab"
        / "cra_ws"
        / "src"
        / "planning"
        / "data"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "input",
            default_value=str(
                data_dir / "raw_workspace.npz"
            ),
            description="Input workspace data",
        ),
        DeclareLaunchArgument(
            "output",
            default_value=str(
                data_dir / "workspace_graph.npz"
            ),
            description="Output GNG graph",
        ),
        ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "planning",
                "generate_gng",
                "--input",
                LaunchConfiguration("input"),
                "--output",
                LaunchConfiguration("output"),
            ],
            output="screen",
        ),
    ])