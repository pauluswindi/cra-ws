from glob import glob
from setuptools import find_packages, setup

package_name = "planning"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cra",
    maintainer_email="pwindikurniawan@gmail.com",
    description="Workspace and whole-arm motion planning for CRA",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "generate_workspace = "
            "planning.workspace.generate_workspace:main",

            "generate_gng = "
            "planning.gng.gng_workspace:main",

            "visualize_workspace = "
            "planning.workspace.visualize_workspace:main",

            "collision_filter = "
            "planning.algorithms.collision_filter:main",

            "planning_visualizer = "
            "planning.visualization.planning_visualizer:main",

            "path_planner = "
            "planning.path_planner:main",

            "planning_node ="
            "planning.planning_node:main",
        ],
    },
)