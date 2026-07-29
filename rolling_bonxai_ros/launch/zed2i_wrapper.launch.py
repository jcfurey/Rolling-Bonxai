"""
ZED2i launch wrapper for Rolling Bonxai.

This launch file wraps the official ZED wrapper launch file and provides
simplified arguments for Rolling Bonxai integration.

Usage:
    ros2 launch rolling_bonxai_ros zed2i_wrapper.launch.py
    ros2 launch rolling_bonxai_ros zed2i_wrapper.launch.py container_name:=my_container
    ros2 launch rolling_bonxai_ros zed2i_wrapper.launch.py use_sim_time:=true sim_mode:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description that wraps official ZED launch."""
    # Declare our arguments
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='zed_container',
        description='Name of the component container for ZED2i'
    )

    enable_ipc_arg = DeclareLaunchArgument(
        'enable_ipc',
        default_value='true',
        description='Enable intra-process communication',
        choices=['true', 'false']
    )

    # Previously hard-coded to false, which made this wrapper unusable against a
    # simulated camera: the ZED nodes stayed on wall time while the rest of the
    # graph followed /clock.
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Drive the ZED wrapper from /clock instead of the wall clock'
    )

    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        choices=['true', 'false'],
        description='Run the ZED wrapper against a simulator instead of real hardware'
    )

    svo_path_arg = DeclareLaunchArgument(
        'svo_path',
        default_value='live',
        description='SVO file to replay, or "live" for a real camera'
    )

    # Get launch configurations
    container_name = LaunchConfiguration('container_name')
    enable_ipc = LaunchConfiguration('enable_ipc')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_mode = LaunchConfiguration('sim_mode')
    svo_path = LaunchConfiguration('svo_path')

    # Path to our custom params file
    zed_params_override = PathJoinSubstitution([
        FindPackageShare('rolling_bonxai_ros'),
        'params',
        'zed2i.yaml'
    ])

    # Include official ZED wrapper launch file
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            # Fixed arguments for ZED2i
            'camera_model': 'zed2i',
            'camera_name': 'zed2i',

            # Container configuration
            'container_name': container_name,
            'enable_ipc': enable_ipc,

            # Override with our params
            'ros_params_override_path': zed_params_override,

            # Reasonable defaults
            'publish_urdf': 'true',
            'publish_tf': 'true',
            'publish_map_tf': 'false',  # Let your SLAM handle this
            'publish_imu_tf': 'false',

            # Live camera by default; set svo_path to replay a recording
            'svo_path': svo_path,
            'publish_svo_clock': 'false',

            # Simulation
            'use_sim_time': use_sim_time,
            'sim_mode': sim_mode,

        }.items()
    )

    return LaunchDescription([
        # Our arguments
        container_name_arg,
        enable_ipc_arg,
        use_sim_time_arg,
        sim_mode_arg,
        svo_path_arg,

        # Include official launch
        zed_wrapper_launch,
    ])
