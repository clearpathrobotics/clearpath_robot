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
    launch_file_axis_camera = PathJoinSubstitution([
        pkg_clearpath_sensors, 'launch', 'axis_camera.launch.py'])

    # Include launch files
    launch_axis_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_axis_camera]),
        launch_arguments=
            [
                (
                    'parameters'
                    ,
                    '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a200/sensors/config/camera_2.yaml'
                )
                ,
                (
                    'namespace'
                    ,
                    'a201_0000/sensors/camera_2'
                )
                ,
                (
                    'robot_namespace'
                    ,
                    'a201_0000'
                )
                ,
            ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_axis_camera)
    return ld
