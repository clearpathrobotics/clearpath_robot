# multi_puma_node

`multi_puma_node` (`MultiPumaNode`) is the ROS 2 node that drives a set of Puma motor controllers over CAN. It is used by Puma-based platforms (Dingo, Ridgeback), runs as `puma_control` in the platform namespace, and is started by `clearpath-platform.service` with parameters from `/etc/clearpath/platform/config/control.yaml`.

It sits between the `ros2_control` Puma hardware interface (`clearpath_hardware_interfaces/src/puma`) and the CAN bus (`vcan0`, bridged to the MCU, which forwards frames to the physical motor bus).

## Terminology

| Term | Meaning in this document |
| --- | --- |
| **Puma** | The physical motor controller board, one per wheel motor. It runs its own speed loop and talks over CAN. |
| **Driver** | The software object `puma_motor_driver::Driver`. There is one per Puma; it speaks that Puma's CAN protocol and keeps its latest values. It is *not* a Linux/kernel driver. |
| **Joint** | A wheel joint from the robot description (e.g. `front_left_wheel_joint`). Each joint maps to one Puma and one Driver. |
| **Device number / CAN ID** | The Puma's address on the CAN bus (`joint_can_ids`). It is the low 6 bits of every CAN frame ID sent to or from that Puma. |
| **N** | The number of joints (= Pumas = Drivers). |
| **MCU** | The platform microcontroller. It bridges `vcan0` on the PC to the physical CAN bus over UDP. |
| **Configuring / configured** | Startup phase in which each Puma is set up (encoder, control mode, gains) and the settings are read back. A Driver is *configured* once all settings are verified. |
| **Active** | The node's normal operating state: every Driver is configured, so commands are sent and feedback is published. |
| **Request / reply** | A request is an empty CAN frame asking a Puma for one value; the reply is the Puma's answer with data. |
| **Polling** | Sending requests periodically. The node polls each Puma for its power status and setpoint. |
| **Unsolicited messages** | Frames the Puma sends on its own without a request: feedback values at ~25 Hz, and fault/temperature/mode at 2 Hz. |
| **Round-robin** | Taking turns: one Puma per tick, in order, then starting over. |
| **Setpoint** | The target value the Puma is currently controlling to (the commanded speed in speed mode). |
| **Power flag** | A Puma status bit that is set after the Puma powers up or resets. If it is set while active, the Puma lost its configuration and must be configured again. |
| **Feedback** | Fast-changing values: duty cycle, current, position, speed, setpoint (`platform/motors/feedback`). |
| **Status** | Slower values: bus/output voltage, temperature, mode, fault flags (`platform/motors/status`). |
| **Fault flags** | Bits reported by the Puma: current, temperature, bus voltage, and bridge driver (the H-bridge gate driver chip). |
| **Frequency check** | A `diagnostic_updater::FrequencyStatus` that counts messages over a window and warns if the rate is outside ±10% of the expected value. |

## Components

| Component | Role |
| --- | --- |
| `MultiPumaNode` | `rclcpp::Node`. Owns the drivers, the CAN interface, timers, publishers, subscriber and diagnostics. |
| `puma_motor_driver::Driver` | One instance per joint. Encodes/decodes the Puma CAN protocol, runs the configuration state machine, caches the latest field values, and owns two `FrequencyStatus` diagnostics. |
| `can_hardware::drivers::SocketCanDriver` | Shared SocketCAN interface. Sends frames for all drivers; delivers received frames to `MultiPumaNode::frameCallback`. |
| `diagnostic_updater::Updater` | Publishes one diagnostic status per driver on `diagnostics` (hardware ID `Puma`). |

## Parameters

| Name | Default | Description |
| --- | --- | --- |
| `canbus_dev` | `vcan0` | SocketCAN interface. |
| `frequency` | `20` | Control loop rate (Hz) and per-driver polling rate. |
| `encoder_cpr` | `1024` | Encoder counts per revolution. |
| `gear_ratio` | `24.0` | Gear ratio, multiplied by each joint's direction. |
| `gain.p` / `gain.i` / `gain.d` | `0.1` / `0.01` / `0.0` | Speed-loop PID gains written to every driver. |
| `joint_names` / `joint_can_ids` / `joint_directions` | `[]` | Per-joint name, CAN device number, and direction (`1` / `-1`). Lengths must match. |

## Topics

| Topic | Direction | Type | Notes |
| --- | --- | --- | --- |
| `platform/motors/cmd` | sub | `sensor_msgs/JointState` | Velocity per joint name; sent as speed (or duty-cycle) commands. Ignored until all drivers are configured. |
| `platform/motors/feedback` | pub | `clearpath_motor_msgs/PumaMultiFeedback` | Duty cycle, current, position, speed, setpoint per driver. |
| `platform/motors/status` | pub | `clearpath_motor_msgs/PumaMultiStatus` | Bus/output voltage, temperature, mode, fault flags per driver. |
| `diagnostics` | pub | `diagnostic_msgs/DiagnosticArray` | `Puma Motor Driver N (<joint>)` entries, 1 Hz. |

## CAN message IDs

All names and values come from [include/puma_motor_driver/can_proto.hpp](../include/puma_motor_driver/can_proto.hpp) (the Stellaris/Jaguar motor-controller protocol). Frames use 29-bit extended IDs:

| Bits | Field | Mask (`can_proto.hpp`) | Value for Puma |
| --- | --- | --- | --- |
| 28-24 | Device type | `CAN_MSGID_DTYPE_M` | `CAN_MSGID_DTYPE_MOTOR` = `0x02000000` |
| 23-16 | Manufacturer | `CAN_MSGID_MFR_M` | `CAN_MSGID_MFR_LM` = `0x00020000` |
| 15-10 | API class | `CAN_MSGID_API_CLASS_M` | `CAN_API_MC_*` (below) |
| 9-6 | API index | `CAN_MSGID_API_ID_M` | message within the class |
| 5-0 | Device number | `CAN_MSGID_DEVNO_M` | `joint_can_ids` entry |

So every ID is `0x02020000 | class | (index << 6) | device_number`. The tables below list the base ID (device number 0); add the device number to get the frame ID. For example, `LM_API_STATUS_POWER` for device 2 is `0x02021600 + 2` = `0x02021602`.

A request is the same ID with no data (DLC 0); the reply or command carries data.

### API classes

| Class | Value | Name in code |
| --- | --- | --- |
| Voltage | `0x0000` | `CAN_API_MC_VOLTAGE` |
| Speed | `0x0400` | `CAN_API_MC_SPD` |
| Position | `0x0C00` | `CAN_API_MC_POS` |
| Current | `0x1000` | `CAN_API_MC_ICTRL` |
| Status | `0x1400` | `CAN_API_MC_STATUS` |
| Configuration | `0x1C00` | `CAN_API_MC_CFG` |

### Messages used by the node

| Name | Base ID | Direction | Used for |
| --- | --- | --- | --- |
| `LM_API_VOLT_EN` | `0x02020000` | PC -> Puma | Enter voltage mode (configuring, voltage mode only) |
| `LM_API_VOLT_SET` | `0x02020080` | both | Duty-cycle command; setpoint poll/reply in voltage mode |
| `LM_API_SPD_EN` | `0x02020400` | PC -> Puma | Enter speed mode (configuring: ClosedLoop and ControlMode steps) |
| `LM_API_SPD_SET` | `0x02020480` | both | Speed command (`cmdCallback`); setpoint poll/reply in speed mode |
| `LM_API_SPD_PC` / `_IC` / `_DC` | `0x020204C0` / `0x02020500` / `0x02020540` | both | Write and read back speed P / I / D gains (configuring) |
| `LM_API_SPD_REF` | `0x02020580` | both | Speed reference = quadrature encoder (configuring) |
| `LM_API_POS_REF` | `0x02020D80` | both | Position reference = encoder (configuring) |
| `LM_API_POS_SET` | `0x02020C80` | both | Setpoint poll/reply in position mode |
| `LM_API_ICTRL_SET` | `0x02021080` | both | Setpoint poll/reply in current mode |
| `LM_API_STATUS_VOLTOUT` | `0x02021400` | Puma -> PC | Output duty cycle (feedback), unsolicited ~25 Hz |
| `LM_API_STATUS_VOLTBUS` | `0x02021440` | Puma -> PC | Bus voltage (status) |
| `LM_API_STATUS_CURRENT` | `0x02021480` | Puma -> PC | Motor current (feedback), unsolicited ~25 Hz |
| `LM_API_STATUS_TEMP` | `0x020214C0` | Puma -> PC | Temperature (status), unsolicited 2 Hz |
| `LM_API_STATUS_POS` | `0x02021500` | Puma -> PC | Position (feedback), unsolicited ~25 Hz |
| `LM_API_STATUS_SPD` | `0x02021540` | Puma -> PC | Speed (feedback), unsolicited ~25 Hz |
| `LM_API_STATUS_FAULT` | `0x020215C0` | Puma -> PC | Fault flags (status), unsolicited 2 Hz; ticks the CAN status frequency check |
| `LM_API_STATUS_POWER` | `0x02021600` | both | Power flag: polled by `poll()`; written with `1` to clear it (configuring) |
| `LM_API_STATUS_CMODE` | `0x02021640` | Puma -> PC | Control mode (status), unsolicited 2 Hz |
| `LM_API_STATUS_VOUT` | `0x02021680` | Puma -> PC | Output voltage (status), unsolicited 2 Hz |
| `CPR_API_STATUS_ANALOG` | `0x020217C0` | Puma -> PC | Analog input (status); Clearpath extension |
| `LM_API_CFG_ENC_LINES` | `0x02021C40` | both | Encoder counts per revolution (configuring) |

Rates marked "unsolicited" were measured with `candump` on a Dingo (do150-0006); the others are only sent when requested or were not observed. Position- and current-mode gains (`LM_API_POS_PC`, `LM_API_ICTRL_PC`, ...) follow the same pattern but are only used if the control mode is changed from the default (speed).

Code locations in `can_proto.hpp`: ID masks [L34-L42](../include/puma_motor_driver/can_proto.hpp#L34-L42), API class and index masks [L108-L109](../include/puma_motor_driver/can_proto.hpp#L108-L109), manufacturer [L85](../include/puma_motor_driver/can_proto.hpp#L85), device type [L95](../include/puma_motor_driver/can_proto.hpp#L95), API classes [L118-L126](../include/puma_motor_driver/can_proto.hpp#L118-L126), voltage [L133-L138](../include/puma_motor_driver/can_proto.hpp#L133-L138), speed [L159-L167](../include/puma_motor_driver/can_proto.hpp#L159-L167), position [L187-L195](../include/puma_motor_driver/can_proto.hpp#L187-L195), current [L202-L209](../include/puma_motor_driver/can_proto.hpp#L202-L209), status [L230-L245](../include/puma_motor_driver/can_proto.hpp#L230-L245), configuration [L271-L280](../include/puma_motor_driver/can_proto.hpp#L271-L280).

## Main functionality

The node has two phases: **configuring** (until every driver is set up) and **active** (normal operation).

### 1. Startup (constructor)

- Reads parameters and checks that the joint lists have the same length.
- Opens the SocketCAN interface.
- Creates one `Driver` per joint and sets its encoder CPR, gear ratio, control mode and PID gains.
- Sets each driver's expected CAN rates for diagnostics.
- Starts two timers: `run_timer_` at `frequency` and `poll_timer_` at `frequency x N` (`N` = number of motors).

### 2. Configuration (`run()`, while inactive)

Every cycle, each driver advances one step through its configuration state machine:

`Power flag -> Encoder refs -> Encoder counts -> Closed loop -> Control mode -> P / I / D gains -> Configured`

- `configureParams()` sends the setting for the current step.
- `verifyParams()` checks the motor's reply and moves to the next step.
- Once all drivers reach `Configured`, the node becomes active.

### 3. Polling (`poll()`, while active)

- Each tick requests power status and setpoint from **one** driver, in round-robin order.
- Every driver is still polled at `frequency`, but the requests and replies are spread out in time.
- Why: with CAN auto-retransmission disabled in the MCU firmware, requests sent back to back lost roughly 40-80% of the replies from three of the four Pumas on a Dingo (do150-0006). The pattern fits requests being dropped after losing arbitration to the Pumas' streaming frames, because the MCU never retried them. Enabling auto-retransmission in the MCU brought back-to-back polling to 99.4-100% replies, so the round-robin is no longer required for correctness. It is kept because it spreads the CAN load and protects robots running older MCU firmware.

### 4. Control loop (`run()`, while active)

1. **Power check**: if any driver reports a power reset, all drivers are reset and the node goes back to configuring.
2. **Process CAN frames**: drains the received-frame queue and passes each frame to every driver's `processMessage()`.
3. **Publish**: sends `platform/motors/feedback` once each feedback field has been received from at least one driver, and `platform/motors/status` once every driver has all of its status fields.

### 5. Commands (`cmdCallback()`)

- Matches each `JointState` name to its driver.
- Sends `commandSpeed()` (or `commandDutyCycle()` in voltage mode).
- Ignored until the node is active.

### 6. Diagnostics (`driverDiagnostic()`, 1 Hz)

Each motor reports:

- Configuration state (`WARN` until configured).
- Two frequency checks, combined into one summary:
  - **CAN feedback**: counts the replies to the polled setpoint request (`*_SET` frames) and is expected at `frequency`.
  - **CAN status**: unsolicited fault messages, expected at 2 Hz.
- Electrical values (voltages, current, temperature) and control mode.
- Fault flags; any active fault raises `WARN`.

## Threading

The node spins on a `MultiThreadedExecutor`, but all timers and the subscription share the default mutually exclusive callback group, so `run()`, `poll()`, `cmdCallback()` and the diagnostic updater never run concurrently. `frameCallback()` runs on the SocketCAN receive thread and only pushes into `recv_msg_queue_` under `recv_msg_mutex_`.

## Diagrams

- Static (class/component) view: [multi_puma_node_static.puml](multi_puma_node_static.puml)
- Dynamic views:
  - Startup and configuring (`active_ == false`): [multi_puma_node_startup.puml](multi_puma_node_startup.puml)
  - Runtime (`active_ == true`): [multi_puma_node_runtime.puml](multi_puma_node_runtime.puml)
  - Diagnostics (both phases): [multi_puma_node_diagnostics.puml](multi_puma_node_diagnostics.puml)
