from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='Port for the web server'
    )

    set_port_env = SetEnvironmentVariable(name='PORT', value=LaunchConfiguration('port'))

    server_node = Node(
        package='softenable_display',
        executable='server',
        name='softenable_display_server',
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        set_port_env,
        server_node
    ])
