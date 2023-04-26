from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameters = LaunchConfiguration('parameters')
    namespace = LaunchConfiguration('namespace')

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='platform/sensors/lidar2d_0')

    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('clearpath_sensors'),
          'config',
          'hokuyo_ust10.yaml'
        ]))

    urg_node = Node(
        package='urg_node',
        namespace=namespace,
        executable='urg_node_driver',
        parameters=[parameters],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(arg_namespace)
    ld.add_action(arg_parameters)
    ld.add_action(urg_node)
    return ld
