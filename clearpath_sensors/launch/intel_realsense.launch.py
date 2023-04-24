from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    name = LaunchConfiguration('name')
    parameters = LaunchConfiguration('parameters')
    topic = LaunchConfiguration('topic')

    arg_name = DeclareLaunchArgument(
        'name',
        default_value='intel_realsense')

    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('clearpath_sensors'),
          'config',
          'intel_realsense.yaml'
        ]))

    arg_topic = DeclareLaunchArgument(
        'topic',
        default_value='scan')

    realsense2_camera_node = Node(
        package='realsense2_camera',
        name=name,
        executable='realsense2_camera_node',
        parameters=[parameters],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(arg_name)
    ld.add_action(arg_parameters)
    ld.add_action(arg_topic)
    ld.add_action(realsense2_camera_node)
    return ld
