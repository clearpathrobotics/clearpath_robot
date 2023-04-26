from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    name = LaunchConfiguration('name')
    parameters = LaunchConfiguration('parameters')
    namespace = LaunchConfiguration('namespace')
    topic = LaunchConfiguration('topic')

    arg_name = DeclareLaunchArgument(
        'name',
        default_value='hokuyo_ust10')

    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('clearpath_sensors'),
          'config',
          'hokuyo_ust10.yaml'
        ]))

    arg_topic = DeclareLaunchArgument(
        'topic',
        default_value=[namespace, '/scan'])

    urg_node = Node(
        package='urg_node',
        name=name,
        executable='urg_node_driver',
        remappings=[
          ('scan', topic),
        ],
        parameters=[parameters],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(arg_name)
    ld.add_action(arg_parameters)
    ld.add_action(arg_topic)
    ld.add_action(urg_node)
    return ld
