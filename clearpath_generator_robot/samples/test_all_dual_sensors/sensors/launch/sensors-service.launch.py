from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages

    # Declare launch files
    launch_file_lidar2d_0 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/lidar2d_0.launch.py'
    launch_file_lidar2d_1 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/lidar2d_1.launch.py'
    launch_file_lidar2d_2 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/lidar2d_2.launch.py'
    launch_file_lidar2d_3 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/lidar2d_3.launch.py'
    launch_file_lidar3d_0 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/lidar3d_0.launch.py'
    launch_file_lidar3d_1 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/lidar3d_1.launch.py'
    launch_file_lidar3d_4 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/lidar3d_4.launch.py'
    launch_file_lidar3d_5 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/lidar3d_5.launch.py'
    launch_file_camera_0 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/camera_0.launch.py'
    launch_file_camera_1 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/camera_1.launch.py'
    launch_file_camera_2 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/camera_2.launch.py'
    launch_file_camera_3 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/camera_3.launch.py'
    launch_file_camera_4 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/camera_4.launch.py'
    launch_file_camera_5 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/camera_5.launch.py'
    launch_file_camera_6 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/camera_6.launch.py'
    launch_file_camera_7 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/camera_7.launch.py'
    launch_file_camera_8 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/camera_8.launch.py'
    launch_file_camera_9 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/camera_9.launch.py'
    launch_file_imu_0 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/imu_0.launch.py'
    launch_file_imu_1 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/imu_1.launch.py'
    launch_file_imu_2 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/imu_2.launch.py'
    launch_file_imu_3 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/imu_3.launch.py'
    launch_file_imu_4 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/imu_4.launch.py'
    launch_file_imu_5 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/imu_5.launch.py'
    launch_file_imu_6 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/imu_6.launch.py'
    launch_file_imu_7 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/imu_7.launch.py'
    launch_file_gps_0 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/gps_0.launch.py'
    launch_file_gps_1 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/gps_1.launch.py'
    launch_file_gps_2 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/gps_2.launch.py'
    launch_file_gps_3 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/gps_3.launch.py'
    launch_file_gps_4 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/gps_4.launch.py'
    launch_file_gps_5 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/gps_5.launch.py'
    launch_file_gps_6 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/gps_6.launch.py'
    launch_file_gps_7 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/gps_7.launch.py'
    launch_file_gps_8 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/gps_8.launch.py'
    launch_file_ins_0 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/ins_0.launch.py'
    launch_file_ins_1 = '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_sensors/sensors/launch/ins_1.launch.py'

    # Include launch files
    launch_lidar2d_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar2d_0]),
    )

    launch_lidar2d_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar2d_1]),
    )

    launch_lidar2d_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar2d_2]),
    )

    launch_lidar2d_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar2d_3]),
    )

    launch_lidar3d_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar3d_0]),
    )

    launch_lidar3d_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar3d_1]),
    )

    launch_lidar3d_4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar3d_4]),
    )

    launch_lidar3d_5 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar3d_5]),
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

    launch_camera_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_camera_3]),
    )

    launch_camera_4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_camera_4]),
    )

    launch_camera_5 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_camera_5]),
    )

    launch_camera_6 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_camera_6]),
    )

    launch_camera_7 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_camera_7]),
    )

    launch_camera_8 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_camera_8]),
    )

    launch_camera_9 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_camera_9]),
    )

    launch_imu_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_imu_0]),
    )

    launch_imu_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_imu_1]),
    )

    launch_imu_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_imu_2]),
    )

    launch_imu_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_imu_3]),
    )

    launch_imu_4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_imu_4]),
    )

    launch_imu_5 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_imu_5]),
    )

    launch_imu_6 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_imu_6]),
    )

    launch_imu_7 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_imu_7]),
    )

    launch_gps_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_gps_0]),
    )

    launch_gps_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_gps_1]),
    )

    launch_gps_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_gps_2]),
    )

    launch_gps_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_gps_3]),
    )

    launch_gps_4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_gps_4]),
    )

    launch_gps_5 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_gps_5]),
    )

    launch_gps_6 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_gps_6]),
    )

    launch_gps_7 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_gps_7]),
    )

    launch_gps_8 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_gps_8]),
    )

    launch_ins_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_ins_0]),
    )

    launch_ins_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_ins_1]),
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_lidar2d_0)
    ld.add_action(launch_lidar2d_1)
    ld.add_action(launch_lidar2d_2)
    ld.add_action(launch_lidar2d_3)
    ld.add_action(launch_lidar3d_0)
    ld.add_action(launch_lidar3d_1)
    ld.add_action(launch_lidar3d_4)
    ld.add_action(launch_lidar3d_5)
    ld.add_action(launch_camera_0)
    ld.add_action(launch_camera_1)
    ld.add_action(launch_camera_2)
    ld.add_action(launch_camera_3)
    ld.add_action(launch_camera_4)
    ld.add_action(launch_camera_5)
    ld.add_action(launch_camera_6)
    ld.add_action(launch_camera_7)
    ld.add_action(launch_camera_8)
    ld.add_action(launch_camera_9)
    ld.add_action(launch_imu_0)
    ld.add_action(launch_imu_1)
    ld.add_action(launch_imu_2)
    ld.add_action(launch_imu_3)
    ld.add_action(launch_imu_4)
    ld.add_action(launch_imu_5)
    ld.add_action(launch_imu_6)
    ld.add_action(launch_imu_7)
    ld.add_action(launch_gps_0)
    ld.add_action(launch_gps_1)
    ld.add_action(launch_gps_2)
    ld.add_action(launch_gps_3)
    ld.add_action(launch_gps_4)
    ld.add_action(launch_gps_5)
    ld.add_action(launch_gps_6)
    ld.add_action(launch_gps_7)
    ld.add_action(launch_gps_8)
    ld.add_action(launch_ins_0)
    ld.add_action(launch_ins_1)
    return ld
