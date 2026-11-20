# clearpath_robot

ROS 2 packages for interfacing with Clearpath Platforms (real hardware).

For supported platforms, sensors and manipulators plus additional details, please see: <https://docs.clearpathrobotics.com/docs/ros/>

## Where this fits in the Clearpath ROS 2 stack

`clearpath_robot` is the **on-robot** half of the stack: it runs on the physical platform's
onboard computer, talks to the motor controllers and sensors, and brings the robot up as a set of
`systemd` services. It consumes the description/control assets from `clearpath_common` and the
files generated from `robot.yaml`.

```mermaid
flowchart LR
    yaml["robot.yaml"] --> config["clearpath_config"]
    config --> genr["clearpath_generator_robot<br/>(this repo)"]
    genr --> out["/etc/clearpath<br/>generated launch / params"]
    common["clearpath_common<br/>description + control"] --> bringup
    out --> bringup["systemd services<br/>(clearpath_robot)"]
    bringup --> hw["motor drivers + sensors<br/>(real hardware)"]
```

## Packages

| Package | Description | Key files |
| --- | --- | --- |
| `clearpath_robot` | Metapackage. Bringup **scripts** and the `systemd` **services** that start the robot on boot. | [`scripts/`](clearpath_robot/scripts) (`generate`, `install`, `check`, `grab-diagnostics`, `shutdown.py`, `vcan`), [`services/`](clearpath_robot/services) (`clearpath-robot.service`, `clearpath-platform.service`, `clearpath-sensors.service`, `clearpath-discovery.service`, …) |
| `clearpath_generator_robot` | Generates the robot-side launch and parameter files from the parsed config. Builds on `clearpath_generator_common`. | [`clearpath_generator_robot/`](clearpath_generator_robot) |
| `clearpath_hardware_interfaces` | Platform hardware drivers / `ros2_control` hardware interfaces for the base. | [`clearpath_hardware_interfaces/`](clearpath_hardware_interfaces) |
| `clearpath_motor_drivers` | Low-level motor controller drivers. Contains `lynx_motor_driver` (current platforms) and `puma_motor_driver` (legacy). | [`lynx_motor_driver/`](clearpath_motor_drivers/lynx_motor_driver), [`puma_motor_driver/`](clearpath_motor_drivers/puma_motor_driver) |
| `clearpath_sensors` | Default launch files and parameter configurations for supported sensors. | [`clearpath_sensors/`](clearpath_sensors) |
| `clearpath_tests` | On-robot hardware/functional test suite (see its [README](clearpath_tests/README.md)). | [`clearpath_tests/`](clearpath_tests) |

## Runtime model

The robot boots via `systemd`. `clearpath-robot.service` is the top-level unit that generates the
launch files (via `clearpath_generator_robot`) and starts the child services (`platform`,
`sensors`, `manipulators`, `discovery`, …). The generated output and the source `robot.yaml` live
in `/etc/clearpath` (the *setup path*). Useful commands on a robot:

```bash
sudo systemctl status clearpath-robot     # overall bringup status
sudo systemctl restart clearpath-robot    # regenerate + restart everything
journalctl -u clearpath-platform -f       # follow a single service's logs
ros2 run clearpath_robot generate         # regenerate launch/params from robot.yaml
```

## Build

From your ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Proton Platform Adaptors

The host package `clearpath_robot` owns the Clearpath-specific Proton launch wrapper.
Generated platform launch files include this wrapper, which loads the selected schema
from `clearpath_firmware` and delegates to the generic `proton_ros2_node` launcher.
Firmware continues to own schemas, firmware binaries, and flash tools, but no longer
installs a launch wrapper or depends on the ROS bridge at runtime.

Build and source the host package alongside the compatible firmware schemas and
platform adaptors, then launch the bridge with:

```bash
ros2 launch clearpath_robot proton.launch.py platform:=core namespace:=a300_00001
```

The previous `ros2 launch clearpath_firmware proton.launch.py` entry point has been
removed. Reinstall both packages and regenerate platform launch files when upgrading.

`clearpath_proton_adaptors` is an aggregate metapackage with no shared library.
It installs all five platform plugins through runtime dependencies:

| Plugin package | Installed firmware schema |
| --- | --- |
| `clearpath_proton_core_adaptors` | `core.yaml` |
| `clearpath_proton_dingo_adaptors` | `d1x0.yaml` |
| `clearpath_proton_jackal_adaptors` | `j100.yaml` |
| `clearpath_proton_ridgeback_adaptors` | `r100.yaml` |
| `clearpath_proton_warthog_adaptors` | `w200.yaml` |

Each package generates only its selected schema from the installed
`clearpath_firmware/share/clearpath_firmware/proton` directory at CMake configure
time. Changes to that schema, the installed generator, or its templates trigger
regeneration on build. Generated C++ and plugin XML stay in the build tree.
The runtime bridge loads only the owner package listed in the selected schema's
`adaptor_packages`.

`clearpath_proton_adaptor_support` installs the shared CMake helper and Python
driver. It does not depend on the aggregate or platform packages, so each plugin
can be built and released independently. Release all six new packages alongside
the aggregate and the compatible firmware schemas; no package relies on sibling
source files. Rebuild and reinstall firmware schemas before rebuilding adaptors
after changing a binding.

With the compatible Proton overlay sourced, optionally limit local GNU Make
build concurrency to reduce peak memory usage:

```bash
MAKEFLAGS=-j2 colcon build --packages-up-to clearpath_proton_adaptors --parallel-workers 1
```

For one platform, use `--packages-up-to clearpath_proton_dingo_adaptors` instead.
`MAKEFLAGS=-j2` limits jobs within a Make build, while `--parallel-workers 1`
limits concurrent colcon packages. Neither is a memory cap, and `MAKEFLAGS`
does not limit Ninja builds. Splitting packages permits platform-only builds;
it does not by itself prevent RAM exhaustion when building all platforms.

CI builds through the aggregate and explicitly tests the helper and all platform
packages with one package worker. The workflow does not set `MAKEFLAGS`, so
compiler parallelism within each package is not capped by the workflow. Firmware uses
`CLEARPATH_DEVHOST_TESTS=ON` in CI to install schemas and run host checks without
requiring the STM32 cross-toolchain.

## Notes

- This package assumes it is running on the robot's onboard computer with `/etc/clearpath`
  populated. For a desktop/offboard workflow use `clearpath_desktop`, and for Gazebo use
  `clearpath_simulator`.
- After editing `robot.yaml`, the generated files are stale until you regenerate (restarting
  `clearpath-robot.service` does this for you).

## Documentation

- [Robot installation & services](https://docs.clearpathrobotics.com/docs/ros/installation/robot) — bringup and the `systemd` services.
- [Generators](https://docs.clearpathrobotics.com/docs/ros/config/generators) — how `robot.yaml` becomes launch/param files.
- [Driving / teleoperation](https://docs.clearpathrobotics.com/docs/ros/tutorials/driving) and [controller pairing](https://docs.clearpathrobotics.com/docs/ros/installation/controller).

## Generator Tests

Changes to the generators in this repository (`clearpath_generator_robot`) may affect the
generated output for launch files and parameter files. The
[clearpath_generator_tests](https://github.com/clearpathrobotics/clearpath_generator_tests)
repository versions the expected output and validates it through CI.

Before merging, ensure a corresponding branch with the **same name** exists in
`clearpath_generator_tests` with regenerated samples. See the
[Development Workflow](https://github.com/clearpathrobotics/clearpath_generator_tests#development-workflow)
section of `clearpath_generator_tests` for the full process.
