# import launch
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     return LaunchDescription([
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(
#                     get_package_share_directory('servo'),
#                     'launch', 'servo_launch.py'
#                 )
#             ),
#         ),
#         # IncludeLaunchDescription(
#         #     PythonLaunchDescriptionSource(
#         #         os.path.join(
#         #             get_package_share_directory('sensing'),
#         #             'launch', 'sensing_launch.py'
#         #         )
#         #     ),
#         # ),
#         # IncludeLaunchDescription(
#         #     PythonLaunchDescriptionSource(
#         #         os.path.join(
#         #             get_package_share_directory('control'),
#         #             'launch', 'control_launch.py'
#         #         )
#         #     ),
#         # ),
#     ])
# */

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='servo',  # Nama paket tempat node berada
            executable='servo',  # Nama executable node C++ yang akan dijalankan
            name='servo',  # Nama untuk node yang diluncurkan
            output='screen',  # Menampilkan output di layar terminal
            # parameters=[{
            #     'robot_name': LaunchConfiguration('robot_name'),
            #     # Bisa menambahkan lebih banyak parameter jika diperlukan
            # }],
            # remappings=[('/old_topic', '/new_topic')],  # Opsional: remap topik jika diperlukan
        ),
        Node(
            package='motion',  # Nama paket tempat node berada
            executable='motion',  # Nama executable node C++ yang akan dijalankan
            name='motion',  # Nama untuk node yang diluncurkan
            output='screen',  # Menampilkan output di layar terminal
            # parameters=[{
            #     'robot_name': LaunchConfiguration('robot_name'),
            #     # Bisa menambahkan lebih banyak parameter jika diperlukan
            # }],
            # remappings=[('/old_topic', '/new_topic')],  # Opsional: remap topik jika diperlukan
        ),
    ])
