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
                    '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/config/camera_0.yaml'
                )
                ,
                (
                    'namespace'
                    ,
                    'cpr_generic_e/sensors/camera_0'
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
    ld.add_action(launch_axis_camera)
    return ld
