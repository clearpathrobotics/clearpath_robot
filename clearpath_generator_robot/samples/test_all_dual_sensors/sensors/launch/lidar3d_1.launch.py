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
    launch_file_ouster_os1 = PathJoinSubstitution([
        pkg_clearpath_sensors, 'launch', 'ouster_os1.launch.py'])

    # Include launch files
    launch_ouster_os1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_ouster_os1]),
        launch_arguments=
            [
                (
                    'parameters'
                    ,
                    '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/config/lidar3d_1.yaml'
                )
                ,
                (
                    'namespace'
                    ,
                    'cpr_generic_e/sensors/lidar3d_1'
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

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_ouster_os1)
    return ld
