# Clearpath Tests

This package contains tests for validating hardware & software performance of supported
Clearpath Robotics' platforms.

## Usage

To run the tests, SSH into the robot and run

```bash
ros2 run clearpath_tests all_tests
```

If your `setup_path` is _not_ `/etc/clearpath`, specify the correct path with

```bash
ros2 run clearpath_tests all_tests --ros-args -p setup_path:=/path/to/setup_dir
```

## Pre-test preparation

Tests require that the robot be mobile; ensure that the robot can travel 5m in a straight line
and perform two complete on-the-spot rotations without colliding with anything, pulling cables,
etc....

## Output

The summary of test results is printed to `stdout`, but the full test report will be saved to
`/tmp/clearpath_test_results.YYYYMMDDhhmm.md`.

Markdown (`.md`) files can be opened in any normal text editor, or can be renders with a
[markdown viewer plugin](https://github.com/simov/markdown-viewer) for Chrome or Firefox.