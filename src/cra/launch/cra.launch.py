from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    cra_share = Path(get_package_share_directory("cra"))
    launch_dir = cra_share / "launch"

    planning_data_dir = (
        Path.home() / "living-lab/cra_ws/src/planning/data"
    )

    workspace_path = LaunchConfiguration("workspace_path")
    graph_path = LaunchConfiguration("graph_path")
    goal_index = LaunchConfiguration("goal_index")
    use_sim_time = LaunchConfiguration("use_sim_time")

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_dir / "simulation.launch.py")
        ),
        launch_arguments={
            "workspace_path": workspace_path,
        }.items(),
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

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_dir / "vision.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
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

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use ROS simulation time for Vision",
        ),

        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Open RViz2 with the CRA robot and human collision displays",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(cra_share / "config/cra.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),

        simulation_launch,
        motion_launch,
        planning_launch,
        vision_launch,
    ])
