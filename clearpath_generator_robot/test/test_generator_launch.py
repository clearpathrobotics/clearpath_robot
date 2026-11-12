# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import ast
import os
from pathlib import Path
import shutil

from ament_index_python.packages import get_package_share_directory
from clearpath_config.common.types.exception import (
    UnsupportedAccessoryException,
    UnsupportedPlatformException,
)
from clearpath_generator_robot.launch.generator import RobotLaunchGenerator
import pytest
import yaml


class TestRobotLaunchGenerator:

    @pytest.mark.parametrize('sample,protocol,platform', [
        ('a300_default.yaml', 'proton', 'core'),
        ('dd150_default.yaml', 'proton', 'd1x0'),
        ('j100_default.yaml', 'proton', 'j100'),
        ('r100_default.yaml', 'proton', 'r100'),
        ('w200_default.yaml', 'proton', 'w200'),
        ('a200_default.yaml', 'proton', None),
        ('a300_default.yaml', 'uros', None),
    ])
    def test_proton_launch_owner(self, tmp_path, sample, protocol, platform):
        """Verify host ownership and unchanged Proton arguments."""
        sample_dir = Path(get_package_share_directory('clearpath_config')) / 'sample'
        sample_path = sample_dir / sample
        config = yaml.safe_load(sample_path.read_text())
        config.setdefault('platform', {}).setdefault('mcu', {})['protocol'] = protocol
        (tmp_path / 'robot.yaml').write_text(yaml.safe_dump(config))

        generator = RobotLaunchGenerator(str(tmp_path))
        generator.generate_platform()
        generated_path = Path(generator.platform_service_launch_file.get_full_path())
        generated_launch = generated_path.read_text()
        ast.parse(generated_launch)

        assert generator.proton_launch.package.name == 'clearpath_robot'
        assert "FindPackageShare('clearpath_firmware')" not in generated_launch
        if platform is None:
            assert 'proton.launch.py' not in generated_launch
        else:
            assert "FindPackageShare('clearpath_robot')" in generated_launch
            expected_path = "pkg_clearpath_robot, 'launch', 'proton.launch.py'"
            assert expected_path in generated_launch
            assert generator.proton_launch.args == [
                ('namespace', generator.namespace),
                ('platform', platform),
            ]

    def test_samples(self):
        errors = []
        share_dir = get_package_share_directory('clearpath_config')
        sample_dir = os.path.join(share_dir, 'sample')
        for sample in os.listdir(sample_dir):
            # Skip non-yaml files
            if os.path.splitext(sample)[-1].lower() != '.yaml':
                continue
            # Create Clearpath Directory
            src = os.path.join(sample_dir, sample)
            dst = os.path.join(os.environ['HOME'], '.clearpath', 'robot.yaml')
            shutil.rmtree(os.path.dirname(dst), ignore_errors=True)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copy(src, dst)
            # Generate
            try:
                rlg = RobotLaunchGenerator(os.path.dirname(dst))
                rlg.generate()
            except UnsupportedAccessoryException as e:
                print(f'Unsupported accessory: {e}')
            except UnsupportedPlatformException as e:
                print(f'Unsupported platform: {e}')
            except FileNotFoundError as e:
                print(f'File not found: {e}')
            except Exception as e:
                errors.append("Sample '%s' failed to load: '%s'" % (
                    sample,
                    e.args[0],
                ))
        assert not errors, 'Errors: %s' % '\n'.join(errors)
