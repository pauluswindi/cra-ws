from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    motion_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('motion'),
                'launch',
                'motion_container.launch.py'  
            ])
        ),
        launch_arguments={
            'tuningState': 'true'  
        }.items()
    )

    # sensor = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare('sensor'),
    #             'launch',
    #             'sensor.launch.py'
    #         ])
    #     )
    # )

    return LaunchDescription([
        motion_container,
        # sensor,
    ])