from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_microstrain_inertial_driver = FindPackageShare('microstrain_inertial_driver')

    parameters = LaunchConfiguration('parameters')
    namespace = LaunchConfiguration('namespace')

    launch_microstrain_imu = PathJoinSubstitution([
        pkg_microstrain_inertial_driver, 'launch', 'microstrain_launch.py'])

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='platform/sensors/imu_0')

    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('clearpath_sensors'),
          'config',
          'microstrain_imu.yaml'
        ]))

    launch_microstrain_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_microstrain_imu]),
        launch_arguments=[
          ('namespace', namespace),
          ('params_file', parameters)
        ]
    )

    ld = LaunchDescription()
    ld.add_action(arg_namespace)
    ld.add_action(arg_parameters)
    ld.add_action(launch_microstrain_imu)
    return ld
