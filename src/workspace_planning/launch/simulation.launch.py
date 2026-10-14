#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Workspace root
    workspace_dir = Path.home() / "living-lab" / "cra_ws"

    # Source package directory
    package_dir = workspace_dir / "src" / "workspace_planning"

    planning_script = (
        package_dir / "scripts" / "planning" / "planning_node.py"
    )

    simulation_script = (
        package_dir / "scripts" / "mujoco" / "robot_simulation.py"
    )

    # Validate script paths
    if not planning_script.is_file():
        raise FileNotFoundError(
            f"Planning script tidak ditemukan: {planning_script}"
        )

    if not simulation_script.is_file():
        raise FileNotFoundError(
            f"Simulation script tidak ditemukan: {simulation_script}"
        )

    return LaunchDescription([
        ExecuteProcess(
            cmd=["python3", str(planning_script)],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", str(simulation_script)],
            output="screen",
        ),
    ])