from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # AZURE KINECT DRIVER CONFIGURATION
    kinect_config = {
        # Body Tracking (Core of the System)
        'body_tracking_enabled': True,
        'body_tracking_smoothing_factor': 0.3,  # Range: 0.0 (noisy) to 1.0 (laggy). Recommended: 0.3 - 0.5

        # Depth Camera (Most Critical for Accuracy)
        'depth_enabled': True,
        'depth_mode': 'NFOV_2X2BINNED',  # Options: 'NFOV_UNBINNED', 'WFOV_UNBINNED', 'NFOV_2X2BINNED' (lightest for CPU inference)

        # Color Camera (RGB)
        'color_enabled': True,
        'color_resolution': '720P',     # Options: '720P', '1080P', '1440P', '1536P', '2160P', '3072P'
        'color_format': 'bgra',         # Options: 'bgra' (raw/fast), 'jpeg' (compressed/USB friendly)
        'fps': 15,                      # Options: 5, 15, 30. (30 starves CPU body tracking; 15 is sustainable)

        # Point Cloud (3D Room Mapping)
        'point_cloud': False,           # Set True if 3D room mapping is needed (very heavy!)
        'rgb_point_cloud': False,       # Set True if colored point cloud is needed (extremely heavy!)
        'point_cloud_in_depth_frame': True,

        # IMU & Infrared
        'rescale_ir_to_mono8': False,
        'ir_mono8_scaling_factor': 1.0,
        'imu_rate_target': 100,         # 0 = Max (1600Hz). 100 saves CPU; no node consumes IMU here

        # Synchronization & Playback
        'wired_sync_mode': 0,           # 0 = Standalone, 1 = Master, 2 = Subordinate
        'subordinate_delay_off_master_usec': 0,
        'sensor_sn': '',                # Fill the serial number when using multiple Kinects
        'recording_file': '',           # Fill the .mkv file path to play back a recording
        'recording_loop_enabled': False,
    }

    # The driver node is declared here instead of including driver.launch.py,
    # so this launch owns it and can respawn it when the device hiccups.
    kinect_node = Node(
        package='azure_kinect_ros_driver',
        executable='node',
        name='k4a_ros_device_node',
        parameters=[kinect_config],
        respawn=True,
        respawn_delay=3.0,
        output='screen',
    )

    # Static Transform Publisher (Connects Robot Base to Camera)
    # base_link currently coincides with the camera mount. When the robot base
    # frame gets a physical definition, fill the measured values here; the
    # bounds in monitor_node must follow the same convention.
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '0.0', '0.0', '0.0',    # x, y, z (meters) - Match the physical camera position on the robot
            '0.0', '0.0', '0.0',    # roll, pitch, yaw (radians)
            'base_link',            # Parent frame
            'camera_base',          # Child frame (Azure Kinect TF root)
        ],
        output='screen',
    )

    return LaunchDescription([
        kinect_node,
        static_tf_publisher,

        # Vision package nodes
        Node(package='vision', executable='tracker_node', name='tracker_node', output='screen'),
        Node(package='vision', executable='monitor_node', name='monitor_node', output='screen'),
        Node(package='vision', executable='visual_node', name='visual_node', output='screen'),
    ])