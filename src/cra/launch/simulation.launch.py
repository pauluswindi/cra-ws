from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    cra_share = Path(get_package_share_directory("cra"))
    with (cra_share / "config/robot.yaml").open(encoding="utf-8") as file:
        config = yaml.safe_load(file)
    gui_enabled = config.get("simulation", {}).get("target_gui", {}).get(
        "enabled", False,
    )
    if not isinstance(gui_enabled, bool):
        raise ValueError("simulation.target_gui.enabled must be a YAML boolean")

    simulation_node = Node(
        package="simulation",
        executable="robot_simulation",
        name="robot_simulation",
        output="screen",
    )

    actions = [
        DeclareLaunchArgument(
            "workspace_path",
            default_value=str(
                Path.home()
                / "living-lab/cra_ws/src/planning/data/raw_workspace.npz"
            ),
            description="Workspace samples shared by Target GUI and Planning",
        ),
        simulation_node,
    ]
    if gui_enabled:
        actions.append(Node(
            package="simulation",
            executable="target_gui",
            name="cra_target_control",
            arguments=["--workspace", LaunchConfiguration("workspace_path")],
            output="screen",
        ))

    return LaunchDescription(actions)
