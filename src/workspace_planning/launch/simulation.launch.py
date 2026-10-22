#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    # Workspace root
    workspace_dir = Path.home() / "living-lab" / "cra_ws"

    # Source package directory
    package_dir = workspace_dir / "src" / "workspace_planning"

    # Planning node
    planning_script = (
        package_dir / "scripts" / "planning" / "planning_node.py"
    )

    # Robot simulation
    simulation_script = (
        package_dir / "scripts" / "mujoco" / "robot_simulation.py"
    )

    # Obstacle controller
    obstacle_script = (
        package_dir / "scripts" / "planning" / "obstacle_controller.py"
    )

    # Validate script paths
    scripts = {
        "Planning Node": planning_script,
        "Robot Simulation": simulation_script,
        "Obstacle Controller": obstacle_script,
    }

    for name, script_path in scripts.items():
        if not script_path.is_file():
            raise FileNotFoundError(
                f"{name} tidak ditemukan: {script_path}"
            )

    # Launch all nodes
    return LaunchDescription([

        # Planning node
        ExecuteProcess(
            cmd=["python3", str(planning_script)],
            name="planning_node",
            output="screen",
        ),

        # Robot simulation
        ExecuteProcess(
            cmd=["python3", str(simulation_script)],
            name="robot_simulation",
            output="screen",
        ),

        # Obstacle controller
        ExecuteProcess(
            cmd=["python3", str(obstacle_script)],
            name="obstacle_controller",
            output="screen",
        ),

    ])