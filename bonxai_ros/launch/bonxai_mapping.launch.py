from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for BonxaiServer."""
    # Declare launch arguments
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='bonxai_container',
        description='Name of the component container for BonxaiServer'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description=(
            'Drive node timers from /clock instead of the wall clock. Set true '
            'when running against a simulator or a bag replayed with --clock. '
            'Overrides the value in the params file.'
        )
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('bonxai_ros'),
            'params',
            'bonxai_params.yaml'
        ]),
        description='Full path to the BonxaiServer parameters file'
    )

    launch_foxglove_arg = DeclareLaunchArgument(
        'launch_foxglove',
        default_value='true',
        choices=['true', 'false'],
        description=(
            'Start a Foxglove Bridge alongside the server. Set false if '
            'foxglove_bridge is not installed, or if a bridge is already running.'
        )
    )

    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Port for Foxglove Bridge WebSocket server'
    )

    # Get launch configuration
    container_name = LaunchConfiguration('container_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    bonxai_params = LaunchConfiguration('params_file')
    launch_foxglove = LaunchConfiguration('launch_foxglove')
    foxglove_port = LaunchConfiguration('foxglove_port')

    # A LaunchConfiguration is a string substitution. Declaring value_type makes
    # the parameter arrive as a real bool/int rather than relying on launch's
    # type inference, which would set use_sim_time to the *string* "true".
    use_sim_time_param = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}

    # Create BonxaiServer component. The use_sim_time dict is listed after the
    # params file so the launch argument wins over whatever the file says.
    bonxai_component = ComposableNode(
        package='bonxai_ros',
        plugin='Bonxai::BonxaiServer',
        name='bonxai_server_node',
        parameters=[bonxai_params, use_sim_time_param],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # Create container with BonxaiServer
    bonxai_container = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[bonxai_component],
        output='screen',
        emulate_tty=True,
    )

    # Foxglove Bridge (runs as separate node, not in container)
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        condition=IfCondition(launch_foxglove),
        parameters=[
            {'port': ParameterValue(foxglove_port, value_type=int)},
            {'address': '0.0.0.0'},
            {'send_buffer_limit': 10000000},
            use_sim_time_param,
        ],
        output='screen',
    )

    return LaunchDescription([
        container_name_arg,
        use_sim_time_arg,
        params_file_arg,
        launch_foxglove_arg,
        foxglove_port_arg,
        bonxai_container,
        foxglove_bridge,
    ])
