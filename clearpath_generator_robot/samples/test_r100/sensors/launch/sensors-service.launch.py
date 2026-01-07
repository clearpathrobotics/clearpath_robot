from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages

    # Declare launch files
    launch_file_lidar2d_0 = '/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_r100/sensors/launch/lidar2d_0.launch.py'

    # Include launch files
    launch_lidar2d_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar2d_0]),
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_lidar2d_0)
    return ld
