# `battery_state_estimator` tests

Parity tests that spawn the real `battery_state_estimator` node, replay a
`Power` voltage sweep into it, and assert the captured `BatteryState` stream
matches what the node's documented math should produce.

## What runs

| File | Tests | Purpose |
| ---- | ----- | ------- |
| `test_node_parity.py` | 24 (parametrized) | Spawns the estimator via `ros2 run` for every `(platform, battery, configuration)` triple declared in `clearpath_config.common.types.platform.Platform.VALID_BATTERIES`. Uses the **shipped** `config/battery_state_estimator/<battery>.yaml` as the parameter source and overlays only `platform` + `cell.num_series` + `cell.num_parallel`. |
| `test_generated_launch_parity.py` | 1 | Loads the launch file emitted by `clearpath_generator_robot` for the `test_a200` sample, isolates the `battery_state_estimator` Node action and runs it via `LaunchService`. Asserts parity against the same procedural model used by `test_node_parity.py`. Pinned to `a200 / ES20_12C / S2P1`. |

Run via colcon:

```bash
colcon test --packages-select clearpath_hardware_interfaces
colcon test-result --verbose --test-result-base build/clearpath_hardware_interfaces
```

Or directly with pytest:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
cd src/clearpath_robot/clearpath_hardware_interfaces/clearpath_hardware_interfaces/battery_state/test
python3 -m pytest --tb=short
```

## Design

There are **no captured "golden" output files** and **no per-scenario
parameter overlays on disk**. The expected `BatteryState` for each step is
computed in-process by `compute_expected_states()` (in `test_node_parity.py`),
which mirrors the node's rolling-average + LUT-interpolation + FULL-status
math. The shipped per-battery yaml is the sole source of truth for LUT,
capacity, voltage, and `power_supply_technology`.

This collapses what used to be three parallel fixture trees
(`fixtures/inputs/`, `fixtures/params/`, `fixtures/goldens/`) into a single
`fixtures/inputs/` tree with **one yaml per battery model** (8 files total).

## `fixtures/inputs/<battery>.yaml` schema

Each input fixture is a **platform-agnostic, configuration-agnostic** voltage
sweep expressed in per-battery-unit volts. The test scales each sample by
`num_series` and places it in the appropriate `Power.measured_voltages[]`
slot at runtime, so one file covers every `(platform, configuration)`
parametrization for that battery.

| Field | Type | Meaning |
| ----- | ---- | ------- |
| `voltage_sweep_per_unit` | `list[float]` | Per-battery-unit voltages walked through in order. Test scales each entry by `num_series` to produce pack voltage. The default sweep contains one sample below the LUT minimum (for the low-clamp branch), every LUT corner, the midpoint between consecutive corners, and one sample above the LUT maximum (for the high-clamp branch). |
| `charger_on_steps`       | `int`         | Number of trailing samples published with `charger_connected = 1` at the highest sweep voltage. Used to exercise the `DISCHARGING -> CHARGING -> FULL` transition (FULL is asserted when charging AND `percentage == 1.0`). |
| `step_period_ns`         | `int`         | Spacing between consecutive `Power.header.stamp` timestamps in nanoseconds. Determines the stamp on each emitted message; does not introduce real-time delay. |
| `start_sec`              | `int`         | First `header.stamp.sec` value. Subsequent stamps advance by `step_period_ns`. |

## Adding a new battery

1. **Add the shipped config.** Drop a new
   `clearpath_hardware_interfaces/config/battery_state_estimator/<new_battery>.yaml`
   following the same `/**: { ros__parameters: ... }` shape as the existing
   ones. It must declare `technology`, `cell.{voltage, capacity}`, and
   `lookup_table.{voltages, percentages}`. Do **not** declare `platform`,
   `cell.num_series`, `cell.num_parallel`, or `rolling_average_window` here —
   those are platform/test-overlaid.

2. **Register the battery in `clearpath_config`.** Make sure the battery
   model appears in the relevant platform's `VALID_BATTERIES` mapping with
   the configurations (`S1P1`, `S2P1`, …) it supports. The test discovers
   scenarios from this mapping at collection time.

3. **Wire the battery into the test.** Edit `test_node_parity.py` and add an
   entry to `BATTERY_TO_CONFIG_FILE`:

   ```python
   BATTERY_TO_CONFIG_FILE: dict[str, str] = {
       ...
       '<MODEL_NAME_AS_IN_VALID_BATTERIES>': '<new_battery>.yaml',
   }
   ```

   Any `VALID_BATTERIES` entry whose model is not in this mapping is silently
   skipped, so the test won't pick up your new battery until you add it here.

4. **Generate the sweep fixture.** Edit
   [tools/generate_inputs.py](tools/generate_inputs.py) and add a new entry
   to `BATTERY_LUTS` mapping the battery file stem to its per-unit LUT
   voltages (re-use an existing constant like `SLA_LUT_VOLTAGES` if the
   chemistry/shape matches). Then:

   ```bash
   python3 tools/generate_inputs.py
   ```

   This regenerates `fixtures/inputs/<new_battery>.yaml`. The shipped LUT
   voltages in the config file and the sweep voltages in the generator must
   match — the test scales both by `num_series`, so a mismatch causes the
   below-min/above-max clamp checks to land on the wrong endpoints.

5. **Run the tests** and confirm the new `(platform, battery, configuration)`
   scenarios appear and pass:

   ```bash
   colcon test --packages-select clearpath_hardware_interfaces
   colcon test-result --verbose --test-result-base build/clearpath_hardware_interfaces
   ```

## Adding a new platform

If you're adding a brand-new platform (not just a new battery for an
existing one), you also need to:

1. Add the platform's `Power` voltage/current index constants and array
   sizes to `PLATFORM_LAYOUTS` in `test_node_parity.py`.
2. Make sure the `initialize_platform_enums` `match` block in the node
   (`battery_state_estimator`) knows about the platform.

The test enumerator (`_iter_scenarios()`) skips any platform that is in
`Platform.VALID_BATTERIES` but not in `PLATFORM_LAYOUTS`, so the test will
silently no-op on your new platform until both sides are wired up.

## Files

```
test/
├── README.md                              (this file)
├── test_node_parity.py                    parametrized 24-scenario parity test
├── test_generated_launch_parity.py        single-scenario launch-file parity test
├── fixtures/
│   └── inputs/
│       ├── dtm8a31.yaml                   one sweep per battery model
│       ├── es20_12c.yaml
│       ├── he2410.yaml
│       ├── he2411.yaml
│       ├── he2613.yaml
│       ├── rb20.yaml
│       ├── tlv1222.yaml
│       └── u1_35.yaml
└── tools/
    └── generate_inputs.py                 regenerates fixtures/inputs/*.yaml
```
