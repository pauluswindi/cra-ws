from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


def generate_launch_description():
    config_file = Path(get_package_share_directory("cra")) / "config/robot.yaml"
    with config_file.open(encoding="utf-8") as file:
        config = yaml.safe_load(file) or {}
    tracker_wait = config.get("vision", {}).get("tracker_enqueue_wait_ms", -1)
    if type(tracker_wait) is not int or tracker_wait not in (-1, 5):
        raise ValueError("vision.tracker_enqueue_wait_ms must be a YAML integer: -1 or 5")
    vision_launch_dir = (
        Path(get_package_share_directory("vision")) / "launch"
    )

    body_tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(vision_launch_dir / "body_tracking.launch.py")
        ),
        launch_arguments={
            "tracker_enqueue_wait_ms": LaunchConfiguration("tracker_enqueue_wait_ms"),
        }.items(),
    )

    reconstruction_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(vision_launch_dir / "reconstruction.launch.py")
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "tracker_enqueue_wait_ms", default_value=str(tracker_wait), choices=["-1", "5"],
            description="Tracker admission wait: -1 blocking or 5 ms; default from robot.yaml"),
        body_tracking_launch,
        reconstruction_launch,
    ])
