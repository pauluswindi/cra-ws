from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    vision_launch_dir = (
        Path(get_package_share_directory("vision")) / "launch"
    )

    body_tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(vision_launch_dir / "body_tracking.launch.py")
        )
    )

    reconstruction_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(vision_launch_dir / "reconstruction.launch.py")
        )
    )

    return LaunchDescription([
        body_tracking_launch,
        reconstruction_launch,
    ])