# Contributing to clearpath_robot

Thanks for your interest in improving `clearpath_robot`! This is the **on-robot** half of the
stack: it runs on the physical platform's onboard computer, talks to the motor controllers and
sensors, and brings the robot up as a set of `systemd` services. Because it drives real hardware,
please read the notes below before opening a pull request.

## Getting started

1. Fork the repository and clone your fork.
2. Create a feature branch off `jazzy`:

   ```bash
   git checkout -b my-feature jazzy
   ```

3. Build the workspace and source it:

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```

4. Install the pre-commit hooks (one-time setup):

   ```bash
   pip install pre-commit
   pre-commit install
   ```

## Linting

This repository uses [pre-commit](https://pre-commit.com/) to run linting and formatting checks
(trailing whitespace, end-of-file, YAML/JSON checks, `markdownlint`, `flake8`, and `cspell`
spell-checking) before each commit. Run them against the whole tree before pushing:

```bash
pre-commit run --all-files
```

## Where things live

See the [Packages section of the README](README.md#packages) for the full map:

- [`clearpath_robot`](clearpath_robot) — bringup scripts and the `systemd` services that start the
  robot on boot.
- [`clearpath_generator_robot`](clearpath_generator_robot) — generates the robot-side launch and
  parameter files (builds on `clearpath_generator_common`).
- [`clearpath_hardware_interfaces`](clearpath_hardware_interfaces) and
  [`clearpath_motor_drivers`](clearpath_motor_drivers) — hardware and motor controller drivers.
- [`clearpath_sensors`](clearpath_sensors) — default sensor launch/param files.
- [`clearpath_tests`](clearpath_tests) — on-robot hardware/functional test suite.

## Testing your changes

Most of this repository interacts with real hardware and the `systemd` bringup, so validate changes
on a robot where possible:

```bash
sudo systemctl restart clearpath-robot     # regenerate + restart everything
sudo systemctl status clearpath-robot      # overall bringup status
journalctl -u clearpath-platform -f        # follow a single service's logs
ros2 run clearpath_robot generate          # regenerate launch/params from robot.yaml
```

The [`clearpath_tests`](clearpath_tests) package provides an on-robot functional test suite — see
its [README](clearpath_tests/README.md). Build and run any package unit tests through `colcon`:

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```

Be careful with changes that move the platform — ensure the area is clear and an e-stop is within
reach.

## Generator tests

Changes to the generators in this repository (`clearpath_generator_robot`) may affect the generated
output for launch and parameter files. The
[clearpath_generator_tests](https://github.com/clearpathrobotics/clearpath_generator_tests)
repository versions the expected output and validates it through CI.

Before merging, ensure a corresponding branch with the **same name** exists in
`clearpath_generator_tests` with regenerated samples. See the
[Development Workflow](https://github.com/clearpathrobotics/clearpath_generator_tests#development-workflow)
section of that repository for the full process.

## Submitting a pull request

1. Make sure the workspace builds and any tests pass.
2. Push your branch and open a pull request against `jazzy`.
3. Write a clear description of what the change does and the platform/hardware you tested against.
4. If your change touches generator output, note the matching `clearpath_generator_tests` branch.

## Reporting issues

Please open issues on the
[GitHub issue tracker](https://github.com/clearpathrobotics/clearpath_robot/issues) and fill out
the bug report template, which walks you through the details we need to reproduce the problem.

## License

By contributing, you agree that your contributions will be licensed under the
[BSD-3-Clause license](LICENSE) that covers this project.
