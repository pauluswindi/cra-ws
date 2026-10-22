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
            "workspace",
            default_value=str(data_dir / "raw_workspace.npz"),
            description="Workspace NPZ file",
        ),
        DeclareLaunchArgument(
            "graph",
            default_value=str(data_dir / "workspace_graph.npz"),
            description="GNG graph NPZ file",
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="both",
            description="Visualization mode: workspace, gng, or both",
        ),
        DeclareLaunchArgument(
            "max_points",
            default_value="10000",
            description="Maximum number of workspace points to display",
        ),
        DeclareLaunchArgument(
            "save",
            default_value="",
            description="Optional output image path",
        ),
        ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "planning",
                "visualize_workspace",
                "--workspace",
                LaunchConfiguration("workspace"),
                "--graph",
                LaunchConfiguration("graph"),
                "--mode",
                LaunchConfiguration("mode"),
                "--max-points",
                LaunchConfiguration("max_points"),
            ],
            output="screen",
        ),
    ])