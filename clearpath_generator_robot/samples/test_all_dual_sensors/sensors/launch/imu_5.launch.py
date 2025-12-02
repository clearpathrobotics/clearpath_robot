from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages
    pkg_clearpath_sensors = FindPackageShare('clearpath_sensors')

    # Declare launch files
    launch_file_phidgets_spatial = PathJoinSubstitution([
        pkg_clearpath_sensors, 'launch', 'phidgets_spatial.launch.py'])
    launch_file_imu_filter = PathJoinSubstitution([
        pkg_clearpath_sensors, 'launch', 'imu_filter.launch.py'])

    # Include launch files
    launch_phidgets_spatial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_phidgets_spatial]),
        launch_arguments=
            [
                (
                    'parameters'
                    ,
                    '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/config/imu_5.yaml'
                )
                ,
                (
                    'namespace'
                    ,
                    'cpr_generic_e/sensors/imu_5'
                )
                ,
                (
                    'robot_namespace'
                    ,
                    'cpr_generic_e'
                )
                ,
            ]
    )

    launch_imu_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_imu_filter]),
        launch_arguments=
            [
                (
                    'namespace'
                    ,
                    'cpr_generic_e/sensors/imu_5'
                )
                ,
                (
                    'parameters'
                    ,
                    '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/config/imu_5.yaml'
                )
                ,
                (
                    'container'
                    ,
                    'imu_filter_container'
                )
                ,
                (
                    'filter'
                    ,
                    'madgwick'
                )
                ,
                (
                    'input_raw'
                    ,
                    'data_raw'
                )
                ,
                (
                    'input_mag'
                    ,
                    'mag'
                )
                ,
                (
                    'output'
                    ,
                    'data'
                )
                ,
            ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_phidgets_spatial)
    ld.add_action(launch_imu_filter)
    return ld
