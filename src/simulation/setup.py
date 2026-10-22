from setuptools import find_packages, setup

package_name = "simulation"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CRA Developer",
    maintainer_email="pwindikurniawan@gmail.com",
    description="MuJoCo simulation interface for Cognitive Robotic Arm",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "robot_simulation = simulation.robot_simulation:main",
        ],
    },
)