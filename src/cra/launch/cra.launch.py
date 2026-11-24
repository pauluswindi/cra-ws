from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import yaml


def configure_latest_log(context, *, settings, helper):
    import runpy

    return runpy.run_path(helper)["logging_actions"](settings)


def generate_launch_description():
    cra_share = Path(get_package_share_directory("cra"))
    launch_dir = cra_share / "launch"

    planning_data_dir = (
        Path.home() / "living-lab/cra_ws/src/planning/data"
    )

    with (cra_share / "config/robot.yaml").open(encoding="utf-8") as file:
        config = yaml.safe_load(file) or {}
    rviz_enabled = config.get("rviz", {}).get("enabled", False)
    if not isinstance(rviz_enabled, bool):
        raise ValueError("rviz.enabled must be a YAML boolean")
    tracker_wait = config.get("vision", {}).get("tracker_enqueue_wait_ms", -1)
    if type(tracker_wait) is not int or tracker_wait not in (-1, 5):
        raise ValueError("vision.tracker_enqueue_wait_ms must be a YAML integer: -1 or 5")
    workspace_default = str(Path(config.get("workspace", {}).get(
        "path", str(planning_data_dir / "raw_workspace.npz"))).expanduser())
    graph_default = str(Path(config.get("gng", {}).get(
        "graph_path", str(planning_data_dir / "workspace_graph.npz"))).expanduser())

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
            "tracker_enqueue_wait_ms": LaunchConfiguration("tracker_enqueue_wait_ms"),
        }.items(),
    )

    return LaunchDescription([
        OpaqueFunction(function=configure_latest_log, kwargs={
            "settings": config.get("logging", {}),
            "helper": str(launch_dir / "latest_log.py"),
        }),
        DeclareLaunchArgument(
            "tracker_enqueue_wait_ms",
            default_value=str(tracker_wait),
            choices=["-1", "5"],
            description="Tracker admission wait: -1 blocking or 5 ms; default from robot.yaml",
        ),
        DeclareLaunchArgument(
            "workspace_path",
            default_value=workspace_default,
            description="Path to workspace samples",
        ),

        DeclareLaunchArgument(
            "graph_path",
            default_value=graph_default,
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
            default_value=str(rviz_enabled).lower(),
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
