from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages

    # Declare launch files
    launch_file_lidar2d_0 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a200/sensors/launch/lidar2d_0.launch.py'
    launch_file_lidar2d_1 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a200/sensors/launch/lidar2d_1.launch.py'
    launch_file_lidar3d_0 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a200/sensors/launch/lidar3d_0.launch.py'
    launch_file_camera_0 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a200/sensors/launch/camera_0.launch.py'
    launch_file_camera_1 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a200/sensors/launch/camera_1.launch.py'
    launch_file_camera_2 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a200/sensors/launch/camera_2.launch.py'
    launch_file_imu_0 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a200/sensors/launch/imu_0.launch.py'
    launch_file_gps_0 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a200/sensors/launch/gps_0.launch.py'
    launch_file_gps_1 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a200/sensors/launch/gps_1.launch.py'

    # Include launch files
    launch_lidar2d_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar2d_0]),
    )

    launch_lidar2d_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar2d_1]),
    )

    launch_lidar3d_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar3d_0]),
    )

    launch_camera_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_camera_0]),
    )

    launch_camera_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_camera_1]),
    )

    launch_camera_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_camera_2]),
    )

    launch_imu_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_imu_0]),
    )

    launch_gps_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_gps_0]),
    )

    launch_gps_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_gps_1]),
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_lidar2d_0)
    ld.add_action(launch_lidar2d_1)
    ld.add_action(launch_lidar3d_0)
    ld.add_action(launch_camera_0)
    ld.add_action(launch_camera_1)
    ld.add_action(launch_camera_2)
    ld.add_action(launch_imu_0)
    ld.add_action(launch_gps_0)
    ld.add_action(launch_gps_1)
    return ld
