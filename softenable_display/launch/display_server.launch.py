from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='8383', description='Web server port')

    set_port_env = SetEnvironmentVariable(name='PORT', value=LaunchConfiguration('port'))

    server_node = Node(
        package='softenable_display',
        executable='server',
        name='softenable_display_server',
        output='screen'
    )

    service_node = Node(
        package='softenable_display',
        executable='display_service',
        name='softenable_display_service',
        output='screen',
        parameters=[
            {'server_host': 'localhost'},
            {'server_port': LaunchConfiguration('port')},   # reuse the same port
            {'endpoint': '/update'},
        ],
        arguments=['--ros-args', '--log-level', 'rmw_cyclonedds_cpp:=error']  
    )

    return LaunchDescription([
        port_arg,
        set_port_env,
        server_node,
        service_node,
    ])
