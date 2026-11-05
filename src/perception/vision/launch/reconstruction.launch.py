import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("cra"),
        "config",
        "reconstruction.yaml"
    )

    reconstruction_node = Node(
        package="vision",
        executable="reconstruction_node",
        name="reconstruction_node",
        output="screen",
        parameters=[config_file]
    )

    return LaunchDescription([
        reconstruction_node
    ])