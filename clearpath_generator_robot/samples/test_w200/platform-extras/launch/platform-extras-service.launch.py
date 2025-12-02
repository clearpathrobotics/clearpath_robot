from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages
    pkg_clearpath_common = FindPackageShare('clearpath_common')

    # Declare launch files
    launch_file_platform_extras = PathJoinSubstitution([
        pkg_clearpath_common, 'launch', 'platform_extras.launch.py'])

    # Include launch files
    launch_platform_extras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_platform_extras]),
        launch_arguments=
            [
                (
                    'setup_path'
                    ,
                    '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_w200'
                )
                ,
                (
                    'use_sim_time'
                    ,
                    'false'
                )
                ,
                (
                    'namespace'
                    ,
                    'w200_0000'
                )
                ,
            ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_platform_extras)
    return ld
