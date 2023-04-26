from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameters = LaunchConfiguration('parameters')
    namespace = LaunchConfiguration('namespace')

    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('clearpath_sensors'),
          'config',
          'intel_realsense.yaml'
        ]))

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='platform/sensors/camera_0')

    realsense2_camera_node = Node(
        package='realsense2_camera',
        namespace=namespace,
        executable='realsense2_camera_node',
        parameters=[parameters],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(arg_parameters)
    ld.add_action(arg_namespace)
    ld.add_action(realsense2_camera_node)
    return ld
