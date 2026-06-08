#!/usr/bin/env python3
"""Parity test: reworked battery_state_estimator vs. its own deterministic math.

This test exercises every ``(platform, battery, configuration)`` triple
supported by ``clearpath_config``'s ``Platform.VALID_BATTERIES`` and whose
battery has a shipped ``config/battery_state_estimator/<battery>.yaml``. For
each scenario it:

1. Spawns the reworked ``battery_state_estimator`` node as a subprocess with
   the shipped per-battery config plus a tiny tmp overlay that sets
   ``platform``, ``cell.num_series`` and ``cell.num_parallel``.
2. Builds a ``Power``-message sweep from the per-battery sweep fixture under
   ``test/fixtures/inputs/<battery>.yaml``, scaled by ``num_series`` and
   placed at the platform's voltage/current indices.
3. Replays the sweep on ``platform/mcu/status/power`` and captures every
   published ``BatteryState`` on ``platform/bms/state``.
4. Computes the expected ``BatteryState`` for each step by mirroring the
   node's math (rolling-average voltage -> LUT interpolation -> percentage,
   etc.) and asserts the captured stream matches step-by-step.

The shipped config files are the sole source of truth for LUT, cell capacity,
cell voltage and technology. No captured golden data is used.

The helper symbols at module scope (``PLATFORM_LAYOUTS``, ``_Harness``,
``_publish_and_wait``, etc.) are imported by ``test_generated_launch_parity.py``.

Run manually::

    pytest -s test_node_parity.py
    pytest -s test_node_parity.py -k a200_ES20_12C_S2P1
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import tempfile
import time
from math import nan
from pathlib import Path

import pytest
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)

from clearpath_platform_msgs.msg import Power
from sensor_msgs.msg import BatteryState


# --------------------------------------------------------------------------- #
# Paths and constants
# --------------------------------------------------------------------------- #

THIS_DIR = Path(__file__).resolve().parent
FIXTURES_DIR = THIS_DIR / 'fixtures'
INPUTS_DIR = FIXTURES_DIR / 'inputs'

ROS_DOMAIN_ID = '91'
PACKAGE = 'clearpath_hardware_interfaces'
EXECUTABLE = 'battery_state_estimator'

STARTUP_TIMEOUT_S = 30.0
SETTLE_S = 1.0
STEP_TIMEOUT_S = 5.0

# Float tolerance for direct numeric fields and array elements. BatteryState
# message fields are float32, so quantization can introduce relative errors of
# up to ~2^-23 (~1.2e-7) per stored value. We pick rel_tol=1e-5 with a small
# absolute floor so values near zero (e.g. percentage just above a LUT corner)
# still compare cleanly without masking real bugs.
FLOAT_REL_TOL = 1e-5
FLOAT_ABS_TOL = 1e-7

# sensor_msgs/BatteryState enum constants used by the expected-state math.
STATUS_CHARGING = BatteryState.POWER_SUPPLY_STATUS_CHARGING
STATUS_DISCHARGING = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
STATUS_FULL = BatteryState.POWER_SUPPLY_STATUS_FULL
HEALTH_GOOD = BatteryState.POWER_SUPPLY_HEALTH_GOOD


def _find_workspace_src(start: Path) -> Path | None:
    """Walk up from ``start`` looking for the workspace ``src/`` directory."""
    for ancestor in [start, *start.parents]:
        if ancestor.name == 'src' and ancestor.is_dir():
            return ancestor
    return None


_WS_SRC = _find_workspace_src(THIS_DIR)
SHIPPED_CONFIG_DIR = (
    _WS_SRC / 'clearpath_robot' / 'clearpath_hardware_interfaces'
    / 'config' / 'battery_state_estimator'
) if _WS_SRC is not None else None


# --------------------------------------------------------------------------- #
# Per-platform Power-message layout (mirrors the node and Power.msg)
# --------------------------------------------------------------------------- #

class PlatformLayout:
    """How to fill in ``measured_voltages``/``measured_currents`` for a platform."""

    def __init__(
        self,
        voltage_index: int,
        current_indices: list[int],
        voltage_array_size: int,
        current_array_size: int,
        current_values: list[float],
    ) -> None:
        if len(current_indices) != len(current_values):
            raise ValueError('current_indices and current_values must align')
        self.voltage_index = voltage_index
        self.current_indices = current_indices
        self.voltage_array_size = voltage_array_size
        self.current_array_size = current_array_size
        self.current_values = current_values


# Index constants and array sizes come from clearpath_platform_msgs/msg/Power.msg.
PLATFORM_LAYOUTS: dict[str, PlatformLayout] = {
    'a200': PlatformLayout(
        voltage_index=0,
        current_indices=[0, 1, 2],
        voltage_array_size=3,
        current_array_size=3,
        current_values=[0.3, 0.4, 0.5],
    ),
    'j100': PlatformLayout(
        voltage_index=0, current_indices=[0],
        voltage_array_size=3, current_array_size=4,
        current_values=[1.0],
    ),
    'w200': PlatformLayout(
        voltage_index=0, current_indices=[0],
        voltage_array_size=4, current_array_size=4,
        current_values=[1.0],
    ),
    'dd100': PlatformLayout(
        voltage_index=0, current_indices=[0],
        voltage_array_size=3, current_array_size=2,
        current_values=[1.0],
    ),
    'do100': PlatformLayout(
        voltage_index=0, current_indices=[0],
        voltage_array_size=3, current_array_size=2,
        current_values=[1.0],
    ),
    'dd150': PlatformLayout(
        voltage_index=0, current_indices=[0],
        voltage_array_size=3, current_array_size=2,
        current_values=[1.0],
    ),
    'do150': PlatformLayout(
        voltage_index=0, current_indices=[0],
        voltage_array_size=3, current_array_size=2,
        current_values=[1.0],
    ),
    'r100': PlatformLayout(
        voltage_index=0, current_indices=[0],
        voltage_array_size=7, current_array_size=1,
        current_values=[1.0],
    ),
}

# S{n}P{m} -> (series, parallel). Mirrors the pre-rework BaseBattery.CONFIGURATIONS.
CONFIGURATIONS: dict[str, tuple[int, int]] = {
    'S1P1': (1, 1), 'S1P2': (1, 2), 'S1P3': (1, 3), 'S1P4': (1, 4),
    'S2P1': (2, 1), 'S4P1': (4, 1), 'S4P3': (4, 3),
}

# Battery model -> shipped config filename under config/battery_state_estimator/.
BATTERY_TO_CONFIG_FILE: dict[str, str] = {
    'HE2613': 'he2613.yaml',
    'HE2411': 'he2411.yaml',
    'HE2410': 'he2410.yaml',
    'RB20': 'rb20.yaml',
    'ES20_12C': 'es20_12c.yaml',
    'U1_35': 'u1_35.yaml',
    'TLV1222': 'tlv1222.yaml',
    '8A31DTM': 'dtm8a31.yaml',
}


# --------------------------------------------------------------------------- #
# Scenario enumeration
# --------------------------------------------------------------------------- #

def _iter_scenarios() -> list[tuple[str, str, str, str]]:
    """Return ``(platform, battery, configuration, scenario_id)`` for every supported triple."""
    try:
        from clearpath_config.common.types.platform import Platform
    except ImportError:
        return []

    scenarios: list[tuple[str, str, str, str]] = []
    for platform_name in sorted(Platform.all_names()):
        if platform_name not in PLATFORM_LAYOUTS:
            continue
        platform_cls = Platform.get(platform_name)
        valid_batteries = getattr(platform_cls, 'VALID_BATTERIES', {}) or {}
        for battery_model, configurations in valid_batteries.items():
            if battery_model not in BATTERY_TO_CONFIG_FILE:
                continue
            for configuration in configurations:
                if configuration not in CONFIGURATIONS:
                    continue
                scenario_id = f'{platform_name}_{battery_model}_{configuration}'
                scenarios.append((platform_name, battery_model, configuration, scenario_id))
    return scenarios


SCENARIOS = _iter_scenarios()


# --------------------------------------------------------------------------- #
# Fixture / config loading
# --------------------------------------------------------------------------- #

def battery_config_path(battery: str) -> Path:
    """Absolute path to the shipped ``config/battery_state_estimator/<battery>.yaml``."""
    if SHIPPED_CONFIG_DIR is None:
        raise RuntimeError('workspace src directory not found')
    return SHIPPED_CONFIG_DIR / BATTERY_TO_CONFIG_FILE[battery]


def load_battery_config(battery: str) -> dict:
    """Load the shipped per-battery config and return its ``ros__parameters`` dict."""
    with battery_config_path(battery).open() as f:
        doc = yaml.safe_load(f)
    # Shipped configs use the '/**' wildcard node-name key.
    top = doc.get('/**') or doc.get('battery_state_estimator')
    return top['ros__parameters']


def load_input(battery: str) -> dict:
    """Load the per-battery voltage-sweep fixture under ``fixtures/inputs/``."""
    path = INPUTS_DIR / BATTERY_TO_CONFIG_FILE[battery]
    with path.open() as f:
        return yaml.safe_load(f)


def write_params_overlay(
    dest: Path,
    platform: str,
    num_series: int,
    num_parallel: int,
) -> None:
    """Write a minimal params YAML overriding ``platform`` and cell counts.

    ROS 2 layers ``--params-file`` overlays in the order given, so passing this
    file as the second ``--params-file`` overrides the shipped config's
    ``cell.num_series`` and ``cell.num_parallel`` while inheriting everything
    else (LUT, capacity, voltage, technology).
    """
    overlay = {
        '/**': {
            'ros__parameters': {
                'platform': platform,
                'cell': {
                    'num_series': int(num_series),
                    'num_parallel': int(num_parallel),
                },
            }
        }
    }
    with dest.open('w') as f:
        yaml.dump(overlay, f, sort_keys=False)


# --------------------------------------------------------------------------- #
# Power-message construction
# --------------------------------------------------------------------------- #

def build_power_steps(
    input_fixture: dict,
    layout: PlatformLayout,
    num_series: int,
) -> list[dict]:
    """Build the full Power-step list (discharge sweep + trailing charger-on segment)."""
    sweep_per_unit = list(input_fixture['voltage_sweep_per_unit'])
    charger_on_steps = int(input_fixture.get('charger_on_steps', 5))
    step_period_ns = int(input_fixture.get('step_period_ns', 100_000_000))
    start_sec = int(input_fixture.get('start_sec', 100))

    pack_voltages = [v * num_series for v in sweep_per_unit]
    # The sweep ends with an above-max sample; the top corner is the
    # second-to-last entry. Charger-on samples sit at that top corner.
    top_voltage = pack_voltages[-2] if len(pack_voltages) >= 2 else pack_voltages[-1]

    steps: list[dict] = []
    step_index = 0
    for voltage in pack_voltages:
        steps.append(_build_step(layout, voltage, 0, step_index, start_sec, step_period_ns))
        step_index += 1
    for _ in range(charger_on_steps):
        steps.append(_build_step(layout, top_voltage, 1, step_index, start_sec, step_period_ns))
        step_index += 1
    return steps


def _build_step(
    layout: PlatformLayout,
    voltage: float,
    charger_connected: int,
    step_index: int,
    start_sec: int,
    step_period_ns: int,
) -> dict:
    voltages = [0.0] * layout.voltage_array_size
    voltages[layout.voltage_index] = float(voltage)
    currents = [0.0] * layout.current_array_size
    for idx, value in zip(layout.current_indices, layout.current_values):
        currents[idx] = float(value)
    total_ns = step_period_ns * step_index
    nanosec = total_ns % 1_000_000_000
    sec = start_sec + total_ns // 1_000_000_000
    return {
        'header': {'sec': int(sec), 'nanosec': int(nanosec), 'frame_id': ''},
        'measured_voltages': voltages,
        'measured_currents': currents,
        'charger_connected': int(charger_connected),
    }


def _input_step_to_power(step: dict) -> Power:
    """Build a Power message from one step dict."""
    msg = Power()
    msg.header.stamp.sec = int(step['header']['sec'])
    msg.header.stamp.nanosec = int(step['header']['nanosec'])
    msg.header.frame_id = step['header'].get('frame_id', '')
    msg.measured_voltages = [float(v) for v in step['measured_voltages']]
    msg.measured_currents = [float(c) for c in step['measured_currents']]
    msg.charger_connected = int(step['charger_connected'])
    return msg


# --------------------------------------------------------------------------- #
# Expected-state computation (mirrors the node math)
# --------------------------------------------------------------------------- #

def _lut_interpolate(scaled_lut: list[tuple[float, float]], v: float) -> float:
    """Linearly interpolate ``v`` over a pack-voltage LUT, clamping at the ends."""
    if v <= scaled_lut[0][0]:
        return scaled_lut[0][1]
    prev = scaled_lut[0]
    for entry in scaled_lut[1:]:
        if v < entry[0]:
            return (v - prev[0]) * (entry[1] - prev[1]) / (entry[0] - prev[0]) + prev[1]
        prev = entry
    return scaled_lut[-1][1]


def compute_expected_states(
    power_steps: list[dict],
    battery_params: dict,
    layout: PlatformLayout,
    num_series: int,
    num_parallel: int,
    rolling_average_window: int = 30,
) -> list[dict]:
    """Compute the expected BatteryState dict for each step, mirroring the node."""
    voltages_per_unit = list(battery_params['lookup_table']['voltages'])
    percentages = list(battery_params['lookup_table']['percentages'])
    scaled_lut = list(zip([v * num_series for v in voltages_per_unit], percentages))

    cell_capacity = float(battery_params['cell']['capacity'])
    capacity = float(num_parallel) * cell_capacity
    technology = int(battery_params['technology'])
    num_cells = num_series * num_parallel

    readings: list[dict] = []
    expected: list[dict] = []
    for step in power_steps:
        readings.append(step)
        if len(readings) > rolling_average_window:
            readings.pop(0)

        voltage = float(step['measured_voltages'][layout.voltage_index])
        current = sum(float(step['measured_currents'][i]) for i in layout.current_indices)

        status = STATUS_CHARGING if int(step['charger_connected']) == 1 else STATUS_DISCHARGING

        avg_v = sum(
            float(r['measured_voltages'][layout.voltage_index]) for r in readings
        ) / len(readings)
        percentage = _lut_interpolate(scaled_lut, avg_v)

        if status == STATUS_CHARGING and percentage == 1.0:
            status = STATUS_FULL

        expected.append({
            'header': dict(step['header']),
            'voltage': voltage,
            'temperature': nan,
            'current': current,
            'charge': capacity * percentage,
            'capacity': capacity,
            'design_capacity': capacity,
            'percentage': percentage,
            'power_supply_status': int(status),
            'power_supply_health': int(HEALTH_GOOD),
            'power_supply_technology': technology,
            'present': True,
            'cell_voltage': [voltage / num_series] * num_cells,
            'cell_temperature': [nan] * num_cells,
            'location': '',
            'serial_number': '',
        })
    return expected


# --------------------------------------------------------------------------- #
# Harness / subprocess helpers
# --------------------------------------------------------------------------- #

def _stamp_key(stamp) -> tuple[int, int]:
    return (int(stamp.sec), int(stamp.nanosec))


class _Harness(Node):
    """Publish Power and capture BatteryState messages keyed by header.stamp."""

    def __init__(self, namespace: str = '') -> None:
        super().__init__('battery_parity_harness', namespace=namespace)
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self._pub = self.create_publisher(Power, 'platform/mcu/status/power', reliable_qos)
        self._captured: dict[tuple[int, int], BatteryState] = {}
        self._sub = self.create_subscription(
            BatteryState,
            'platform/bms/state',
            self._on_battery_state,
            qos_profile_sensor_data,
        )

    def _on_battery_state(self, msg: BatteryState) -> None:
        self._captured[_stamp_key(msg.header.stamp)] = msg

    def publisher(self):
        return self._pub

    def subscription_count(self) -> int:
        return self._pub.get_subscription_count()

    def publisher_count(self) -> int:
        return self._sub.get_publisher_count()

    def captured(self) -> dict[tuple[int, int], BatteryState]:
        return self._captured


def _wait_for_estimator(harness: _Harness, timeout_s: float) -> None:
    """Block until discovery is bidirectional, then spin a short settle window."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(harness, timeout_sec=0.1)
        if harness.subscription_count() >= 1 and harness.publisher_count() >= 1:
            break
    else:
        raise TimeoutError(
            f'estimator did not connect within {timeout_s:.1f}s '
            f'(sub_count={harness.subscription_count()}, pub_count={harness.publisher_count()})'
        )
    settle_deadline = time.monotonic() + SETTLE_S
    while time.monotonic() < settle_deadline:
        rclpy.spin_once(harness, timeout_sec=0.05)


def _publish_and_wait(
    harness: _Harness,
    power_msg: Power,
    timeout_s: float,
) -> BatteryState:
    """Publish a single Power message and wait for the BatteryState with the same stamp."""
    key = _stamp_key(power_msg.header.stamp)
    harness.publisher().publish(power_msg)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(harness, timeout_sec=0.05)
        if key in harness.captured():
            return harness.captured()[key]
    raise TimeoutError(f'no BatteryState received for stamp {key} within {timeout_s:.1f}s')


def _start_estimator(param_files: list[Path], env: dict[str, str]) -> subprocess.Popen:
    """Spawn ``ros2 run`` with one or more ``--params-file`` overlays."""
    cmd = ['ros2', 'run', PACKAGE, EXECUTABLE, '--ros-args']
    for pf in param_files:
        cmd += ['--params-file', str(pf)]
    return subprocess.Popen(
        cmd,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _stop_estimator(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=5)


# --------------------------------------------------------------------------- #
# Comparison helpers
# --------------------------------------------------------------------------- #

def _floats_match(actual: float, expected: float) -> bool:
    a_nan = isinstance(actual, float) and math.isnan(actual)
    e_nan = isinstance(expected, float) and math.isnan(expected)
    if a_nan or e_nan:
        return a_nan and e_nan
    return math.isclose(
        float(actual), float(expected),
        rel_tol=FLOAT_REL_TOL, abs_tol=FLOAT_ABS_TOL,
    )


def _battery_state_to_dict(msg: BatteryState) -> dict:
    """Convert a BatteryState to the plain-Python shape used by ``compute_expected_states``."""
    return {
        'header': {
            'sec': int(msg.header.stamp.sec),
            'nanosec': int(msg.header.stamp.nanosec),
            'frame_id': msg.header.frame_id,
        },
        'voltage': float(msg.voltage),
        'temperature': float(msg.temperature),
        'current': float(msg.current),
        'charge': float(msg.charge),
        'capacity': float(msg.capacity),
        'design_capacity': float(msg.design_capacity),
        'percentage': float(msg.percentage),
        'power_supply_status': int(msg.power_supply_status),
        'power_supply_health': int(msg.power_supply_health),
        'power_supply_technology': int(msg.power_supply_technology),
        'present': bool(msg.present),
        'cell_voltage': [float(v) for v in msg.cell_voltage],
        'cell_temperature': [float(t) for t in msg.cell_temperature],
        'location': str(msg.location),
        'serial_number': str(msg.serial_number),
    }


def _compare_step(actual: dict, expected: dict, step_idx: int) -> list[str]:
    """Return a list of human-readable mismatch descriptions (empty on match)."""
    mismatches: list[str] = []

    if actual['header']['sec'] != expected['header']['sec']:
        mismatches.append(
            f'header.sec: actual={actual["header"]["sec"]} expected={expected["header"]["sec"]}'
        )
    if actual['header']['nanosec'] != expected['header']['nanosec']:
        mismatches.append(
            f'header.nanosec: actual={actual["header"]["nanosec"]} '
            f'expected={expected["header"]["nanosec"]}'
        )
    if actual['header']['frame_id'] != expected['header']['frame_id']:
        mismatches.append(
            f'header.frame_id: actual={actual["header"]["frame_id"]!r} '
            f'expected={expected["header"]["frame_id"]!r}'
        )

    for field in ('voltage', 'temperature', 'current', 'charge',
                  'capacity', 'design_capacity', 'percentage'):
        if not _floats_match(actual[field], expected[field]):
            mismatches.append(
                f'{field}: actual={actual[field]!r} expected={expected[field]!r}'
            )

    for field in ('power_supply_status', 'power_supply_health', 'power_supply_technology'):
        if int(actual[field]) != int(expected[field]):
            mismatches.append(
                f'{field}: actual={actual[field]} expected={expected[field]}'
            )
    if bool(actual['present']) != bool(expected['present']):
        mismatches.append(
            f'present: actual={actual["present"]} expected={expected["present"]}'
        )
    for field in ('location', 'serial_number'):
        if actual[field] != expected[field]:
            mismatches.append(
                f'{field}: actual={actual[field]!r} expected={expected[field]!r}'
            )

    for field in ('cell_voltage', 'cell_temperature'):
        a_arr = list(actual[field])
        e_arr = list(expected[field])
        if len(a_arr) != len(e_arr):
            mismatches.append(
                f'{field}: length mismatch actual={len(a_arr)} expected={len(e_arr)}'
            )
            continue
        for i, (av, ev) in enumerate(zip(a_arr, e_arr)):
            if not _floats_match(av, ev):
                mismatches.append(f'{field}[{i}]: actual={av!r} expected={ev!r}')

    return [f'step {step_idx}: {m}' for m in mismatches]


# --------------------------------------------------------------------------- #
# Test
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize(
    'platform,battery,configuration,scenario_id',
    SCENARIOS,
    ids=[s[3] for s in SCENARIOS],
)
def test_battery_state_parity(
    platform: str,
    battery: str,
    configuration: str,
    scenario_id: str,
    tmp_path: Path,
) -> None:
    """End-to-end parity check for one (platform, battery, configuration) scenario."""
    if not SCENARIOS:
        pytest.skip('No scenarios discovered (clearpath_config not importable?)')

    layout = PLATFORM_LAYOUTS[platform]
    num_series, num_parallel = CONFIGURATIONS[configuration]

    base_params = battery_config_path(battery)
    overlay_params = tmp_path / f'overlay_{scenario_id}.yaml'
    write_params_overlay(overlay_params, platform, num_series, num_parallel)

    battery_params = load_battery_config(battery)
    input_fixture = load_input(battery)

    power_steps = build_power_steps(input_fixture, layout, num_series)
    expected_states = compute_expected_states(
        power_steps, battery_params, layout, num_series, num_parallel)
    assert len(power_steps) == len(expected_states)

    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = ROS_DOMAIN_ID
    env['ROS_LOCALHOST_ONLY'] = '1'
    # Mirror isolation onto the in-process rclpy context too.
    os.environ['ROS_DOMAIN_ID'] = ROS_DOMAIN_ID
    os.environ['ROS_LOCALHOST_ONLY'] = '1'

    proc = _start_estimator([base_params, overlay_params], env)
    rclpy.init()
    harness = _Harness()
    captured_dicts: list[dict] = []
    try:
        _wait_for_estimator(harness, STARTUP_TIMEOUT_S)
        for step in power_steps:
            power_msg = _input_step_to_power(step)
            bs = _publish_and_wait(harness, power_msg, STEP_TIMEOUT_S)
            captured_dicts.append(_battery_state_to_dict(bs))
    finally:
        harness.destroy_node()
        rclpy.shutdown()
        _stop_estimator(proc)

    all_mismatches: list[str] = []
    for idx, (actual, expected) in enumerate(zip(captured_dicts, expected_states)):
        all_mismatches.extend(_compare_step(actual, expected, idx))

    if all_mismatches:
        shown = '\n  '.join(all_mismatches[:20])
        more = '' if len(all_mismatches) <= 20 else f'\n  ... and {len(all_mismatches) - 20} more'
        pytest.fail(
            f'{scenario_id}: {len(all_mismatches)} mismatch(es) vs expected:\n  {shown}{more}'
        )
