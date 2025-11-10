"""Scripts to generate samples using the clearpath_generator_robot."""
import argparse
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


def main():
    """Generate all files in directory path from argument."""
    default_root_dir = os.path.join(
        os.path.dirname(
            os.path.dirname(
                os.path.realpath(__file__))),
        'samples')
    # Get Root Directory from Args
    parser = argparse.ArgumentParser(
        prog='Clearpath Robot Sample Generator',
        description='Generate all launch and parameter files '
                    'from test samples in the clearpath_config package.',
    )
    parser.add_argument(
        '--out',
        help='Output directory of generated files.',
        default=default_root_dir,
        required=False)
    args = parser.parse_args()

    root_dir = os.path.abspath(args.out)
    assert os.path.isdir(root_dir), f'Output directory "{root_dir}" does not exist.'

    generate_test_samples(root_dir)


if __name__ == '__main__':
    main()
