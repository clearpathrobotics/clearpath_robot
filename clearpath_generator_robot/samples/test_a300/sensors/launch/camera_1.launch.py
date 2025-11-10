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
    launch_file_luxonis_oakd = PathJoinSubstitution([
        pkg_clearpath_sensors, 'launch', 'luxonis_oakd.launch.py'])

    # Include launch files
    launch_luxonis_oakd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_luxonis_oakd]),
        launch_arguments=
            [
                (
                    'parameters'
                    ,
                    '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a300/sensors/config/camera_1.yaml'
                )
                ,
                (
                    'namespace'
                    ,
                    'a300_00000/sensors/camera_1'
                )
                ,
                (
                    'robot_namespace'
                    ,
                    'a300_00000'
                )
                ,
            ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_luxonis_oakd)
    return ld
