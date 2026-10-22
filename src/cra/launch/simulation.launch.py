from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    simulation_node = Node(
        package="simulation",
        executable="robot_simulation",
        name="robot_simulation",
        output="screen",
    )

    return LaunchDescription([
        simulation_node,
    ])