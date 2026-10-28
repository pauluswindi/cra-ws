from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    motion_node = ComposableNode(
        package="motion",
        plugin="motion::MotionComponent",
        name="motion_component",
    )

    container = ComposableNodeContainer(
        name="motion_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[motion_node],
        output="screen",
    )

    return LaunchDescription([container])