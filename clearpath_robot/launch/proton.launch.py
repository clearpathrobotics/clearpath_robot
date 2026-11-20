from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_clearpath_firmware = FindPackageShare('clearpath_firmware')
    pkg_proton_ros2_node = FindPackageShare('proton_ros2_node')

    # Select the platform schema containing Proton configuration and ROS bindings.
    arg_platform = DeclareLaunchArgument(
        'platform',
        default_value='core',
        choices=['core', 'd1x0', 'j100', 'r100', 'w200'],
        description='Platform to use for Proton communication',
    )

    # Namespace argument
    arg_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Namespace for the node'
    )

    # The platform schema holds both the Proton config and the ROS bindings
    config_file = PathJoinSubstitution([
        pkg_clearpath_firmware,
        'proton',
        [LaunchConfiguration('platform'), '.yaml'],
    ])

    # Include the proton_ros2 node launch with the platform-specific config
    proton_ros2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_proton_ros2_node, 'launch', 'proton_ros2_node.launch.py'])
        ),
        launch_arguments={
            'proton_config_file': config_file,
            'binding_config_file': config_file,
            'namespace': LaunchConfiguration('namespace'),
            'target': 'pc',
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(arg_platform)
    ld.add_action(arg_namespace)
    ld.add_action(proton_ros2_launch)

    return ld
