# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2026, Clearpath Robotics, Inc., All rights reserved.
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

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.

"""Scripts to generate samples using the clearpath_generator_robot."""
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from clearpath_generator_robot.launch.generator import RobotLaunchGenerator
from clearpath_generator_robot.param.generator import RobotParamGenerator


class GenerationFailureException(Exception):
    """Exception to capture generation failures."""

    def __init__(self, message, errors):
        """Initialize default exception and keep errors."""
        super().__init__(message)
        self.errors = errors


def generate_launch(setup_path) -> None:
    """Generate launch files."""
    rlg = RobotLaunchGenerator(setup_path)
    rlg.generate()


def generate_param(setup_path) -> None:
    """Generate parameter files."""
    rpg = RobotParamGenerator(setup_path)
    rpg.generate()


def error_log(name: str, sample: str, error: Exception) -> str:
    """Return error entry."""
    return f'{name} failed for sample "{sample}" with error: \n{error}'


def generate_test_samples(root_dir: str):
    """Generate all files from common generator."""
    # Iterate through all samples in clearpath_config
    share_dir = get_package_share_directory('clearpath_config')
    sample_dir = os.path.join(share_dir, 'sample')
    sample_errors = []
    for sample in os.listdir(sample_dir):
        # Filter for Test Samples
        if 'test' not in sample:
            continue
        print(sample)
        # Create Clearpath Directory
        src = os.path.join(sample_dir, sample)
        dst = os.path.join(
            os.path.join(root_dir,  os.path.splitext(os.path.basename(sample))[0]),
            'robot.yaml')
        setup_path = os.path.dirname(dst)

        shutil.rmtree(setup_path, ignore_errors=True)
        os.makedirs(setup_path, exist_ok=True)
        shutil.copy(src, dst)
        errors = []
        # Launch
        try:
            generate_launch(setup_path)
        except Exception as e:
            errors.append(error_log('RobotLaunchGenerator', sample, e))
        # Param
        try:
            generate_param(setup_path)
        except Exception as e:
            errors.append(error_log('RobotParamGenerator', sample, e))
        if len(errors) > 0:
            sample_errors.append(f'Sample "{sample}" failed to generate:\n''\n  '.join(errors))
    if len(sample_errors) > 0:
        raise GenerationFailureException(
            message=f'Generation failed for {len(sample_errors)} samples:\n'
                    f'{"\n".join(sample_errors)}',
            errors=sample_errors
        )
