from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    cra_share = Path(get_package_share_directory("cra"))

    launch_dir = cra_share / "launch"

    planning_data_dir = Path.home() / "living-lab/cra_ws/src/planning/data"

    workspace_path = LaunchConfiguration("workspace_path")
    graph_path = LaunchConfiguration("graph_path")
    goal_index = LaunchConfiguration("goal_index")

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_dir / "simulation.launch.py")
        )
    )

    motion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_dir / "motion.launch.py")
        )
    )

    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_dir / "planning.launch.py")
        ),
        launch_arguments={
            "workspace_path": workspace_path,
            "graph_path": graph_path,
            "goal_index": goal_index,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "workspace_path",
            default_value=str(
                planning_data_dir / "raw_workspace.npz"
            ),
            description="Path to workspace samples",
        ),
        DeclareLaunchArgument(
            "graph_path",
            default_value=str(
                planning_data_dir / "workspace_graph.npz"
            ),
            description="Path to GNG joint graph",
        ),
        DeclareLaunchArgument(
            "goal_index",
            default_value="100",
            description="Temporary goal configuration index",
        ),
        simulation_launch,
        motion_launch,
        planning_launch,
    ])