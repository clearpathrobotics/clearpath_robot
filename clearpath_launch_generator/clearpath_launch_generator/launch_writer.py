from typing import List

import os

from ament_index_python.packages import get_package_share_directory


class Package():
    def __init__(self, name: str) -> None:
        self.name = name
        self.declaration = 'pkg_' + name

    def find_package_share(self) -> str:
        return 'FindPackageShare(\'{0}\')'.format(self.name)

class LaunchFile():
    def __init__(self, name: str, path: str = 'launch', package: Package = None) -> None:
        self.package = package
        self.path = path
        self.name = 'launch_' + name
        self.declaration = 'launch_file_{0}'.format(name)
        self.file = '{0}.launch.py'.format(name)

    def get_full_path(self):
        if self.package:
            return os.path.join(get_package_share_directory(self.package.name), self.path, self.file)
        else:
            return os.path.join(self.path, self.file)

class ParameterFile():
    def __init__(self, name: str, path: str = 'config', package: Package = None) -> None:
        self.package = package
        self.path = path
        self.name = 'param_file_{0}'.format(name)
        self.file = '{0}.yaml'.format(name)

    def get_full_path(self):
        if self.package:
            return os.path.join(get_package_share_directory(self.package.name), self.path, self.file)
        else:
            return os.path.join(self.path, self.file)


class LaunchWriter():
    tab = '    '

    def __init__(self, launch_file: LaunchFile):
        self.launch_file = launch_file
        self.actions = []
        self.included_packages = []
        self.included_launch_files = []
        self.declared_launch_args = []
        self.file = open(self.launch_file.get_full_path(), 'w+')
        self.initialize_file()

    def write(self, string, indent_level=1):
        self.file.write('{0}{1}\n'.format(self.tab * indent_level, string))

    def write_comment(self, comment, indent_level=1):
        self.write('# {0}'.format(comment), indent_level)

    def write_newline(self):
        self.write('', 0)

    def write_actions(self):
        self.write('ld = LaunchDescription()')
        for action in self.actions:
            self.write('ld.add_action({0})'.format(action))
        self.write('return ld')

    def find_package(self, package: Package):
        if package not in self.included_packages:
            self.write('pkg_{0} = FindPackageShare(\'{0}\')'.format(package.name))
            self.included_packages.append(package)

    def path_join_substitution(package, folder, file):
        return 'PathJoinSubstitution([{0}, \'{1}\', \'{2}\'])'.format(package, folder, file)

    def declare_launch_arg(self, arg, default_value='', description='', launch_config=True):
        if arg not in self.declared_launch_args:
            # Declare launch arg
            self.write('{0}_launch_arg = DeclareLaunchArgument('.format(arg))
            self.write('\'{0}\','.format(arg), indent_level=2)
            self.write('default_value=\'{0}\','.format(default_value), indent_level=2)
            self.write('description=\'{0}\')'.format(description), indent_level=2)
            self.write_newline()

            # Launch configuration
            if launch_config:
                self.write('{0} = LaunchConfiguration(\'{0}\')'.format(arg))
                self.write_newline()

            # Add launch arg to launch description actions
            self.actions.append('{0}_launch_arg'.format(arg))

            self.declared_launch_args.append(arg)

    def declare_launch_file(self, launch_file: LaunchFile):
        if launch_file not in self.included_launch_files:
            if launch_file.package is None:
                self.write('{0} = \'{1}\''.format(launch_file.declaration, os.path.join(launch_file.path, launch_file.file)))
            else:
                self.write('{0} = PathJoinSubstitution(['.format(launch_file.declaration))
                self.write('{0}, \'{1}\', \'{2}\'])'.format(
                    launch_file.package.declaration,
                    launch_file.path,
                    launch_file.file), indent_level=2)
            self.included_launch_files.append(launch_file)

    def include_launch_file(self, launch_file: LaunchFile, args: dict = None):
        self.write('{0} = IncludeLaunchDescription('.format(launch_file.name))
        self.write('PythonLaunchDescriptionSource([{0}]),'.format(launch_file.declaration), indent_level=2)
        if args is not None:
            self.write('launch_arguments=[', indent_level=2)
            for key in args.keys():
                value = args.get(key)
                if isinstance(value, str):
                    self.write('(\'{0}\', \'{1}\'),'.format(key, value), indent_level=3)
                elif isinstance(value, dict):
                    self.write('(\'{0}\', {1}),'.format(key, value), indent_level=3)
                elif isinstance(value, ParameterFile):
                    self.write('(\'{0}\', \'{1}\'),'.format(key, value.get_full_path()), indent_level=3)
            self.write(']', indent_level=2)
        self.write(')')
        self.write_newline()
        self.actions.append(launch_file.name)

    def initialize_file(self):
        self.write('from launch import LaunchDescription', 0)
        self.write('from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument', 0)
        self.write('from launch.launch_description_sources import PythonLaunchDescriptionSource', 0)
        self.write('from launch.substitutions import PathJoinSubstitution, LaunchConfiguration', 0)
        self.write('from launch_ros.substitutions import FindPackageShare', 0)
        self.write_newline()
        self.write_newline()
        self.write('def generate_launch_description():', 0)
        self.write_newline()

    def close_file(self):
        self.write_actions()
        self.file.close()