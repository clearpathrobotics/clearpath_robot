from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    name = LaunchConfiguration('name')
    parameters = LaunchConfiguration('parameters')
    topic = LaunchConfiguration('topic')

    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name=[name, '_driver'],
        parameters=[parameters],
        output='screen'
    )

    velodyne_pointcloud_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_convert_node',
        name=[name, '_pointcloud'],
        output='screen',
        parameters=[parameters]
    )

    velodyne_laserscan_node = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        name=[name, '_laserscan'],
        output='screen',
        parameters=[parameters]
    )

    ld = LaunchDescription()
    ld.add_action(velodyne_driver_node)
    ld.add_action(velodyne_pointcloud_node)
    ld.add_action(velodyne_laserscan_node)
    return ld
