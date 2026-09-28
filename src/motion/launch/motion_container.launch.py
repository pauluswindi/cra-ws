from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    tuning_arg = DeclareLaunchArgument(
        'tuningState',
        default_value='false',
        description='Enable tuning mode for motion control'
    )
    tuning_state = LaunchConfiguration('tuningState')
    
    # # Parameter files (bisa dihapus jika file yaml belum ada atau tidak dibutuhkan saat testing)
    # param_files = [
    #     "/home/eilero/eilero_parameters/motion/posdef.yaml",
    #     "/home/eilero/eilero_parameters/motion/motion_parameters.yaml",
    # ]
    
    motion_container = ComposableNodeContainer(
        name='motion_container',             
        namespace='',                        
        package='rclcpp_components',         
        executable='component_container_mt',   
        composable_node_descriptions=[

            ComposableNode(
                package='servo',                                   
                plugin='servo_driver::ServoComponent',            
                name='servo',                                        
                parameters=[],                                      
                extra_arguments=[{'use_intra_process_comms': False}] 
            ),
            
            ComposableNode(
                package='motion',                                 
                plugin='motion::MotionComponent',                 
                name='motion',                                     
                # parameters=param_files + [{'tuningState': tuning_state}],
                parameters=[],
                extra_arguments=[{'use_intra_process_comms': False}]
            ),
    
        ],
        output='screen',
    )
    
    return LaunchDescription([
        tuning_arg,
        motion_container,
    ])