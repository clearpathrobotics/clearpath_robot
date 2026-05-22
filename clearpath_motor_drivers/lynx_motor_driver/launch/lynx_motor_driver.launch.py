from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  # Packages
  pkg_lynx_motor_driver = FindPackageShare('lynx_motor_driver')

  # Launch Configurations
  parameters = LaunchConfiguration('parameters')
  can_bus = LaunchConfiguration('can_bus')
  namespace = LaunchConfiguration('namespace')

  # Launch Arguments
  arg_parameters = DeclareLaunchArgument(
    'parameters',
    default_value=PathJoinSubstitution([
      pkg_lynx_motor_driver,
      'config',
      'single_test.yaml'
    ]))

  arg_can_bus = DeclareLaunchArgument(
    'can_bus',
    default_value='can0'
  )

  arg_namespace = DeclareLaunchArgument(
    'namespace',
    default_value='',
    description='Robot namespace'
  )

  multi_bldc_group_action = GroupAction(
    actions=[
      PushRosNamespace(namespace),

      Node(
        package='lynx_motor_driver',
        executable='lynx_motor_driver',
        parameters=[
          parameters,
          {'can_bus': can_bus}
        ],
        output='screen'
      )
    ]
  )

  ld = LaunchDescription()
  ld.add_action(arg_parameters)
  ld.add_action(arg_can_bus)
  ld.add_action(arg_namespace)
  ld.add_action(multi_bldc_group_action)
  return ld
