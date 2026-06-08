#!/usr/bin/env python3
"""Generate per-battery voltage-sweep fixtures.

For each battery model supported by the shipped
``config/battery_state_estimator/*.yaml`` files, this script writes one
platform-agnostic input fixture at::

    <pkg>/clearpath_hardware_interfaces/battery_state/test/fixtures/inputs/
        <battery>.yaml

Each fixture contains only the per-battery-unit voltage sweep plus a small
amount of timing/charger metadata. The tests scale the sweep by ``num_series``
and place samples into a platform-specific ``Power`` message at runtime, so a
single fixture covers every ``(platform, configuration)`` parametrization for
that battery.

Sweep shape per battery:

* For each LUT corner ``Ci`` and the midpoint between consecutive corners
  ``Mi``, emit one sample (ascending order).
* Prepend one sample ``OUT_OF_RANGE_MARGIN_V`` below the LUT minimum and
  append one above the maximum so clamp behaviour is exercised.

The test then appends a trailing charger-on segment at the top corner voltage
(see ``charger_on_steps``) to exercise the CHARGING and CHARGING -> FULL
transitions.

Usage::

    python3 generate_inputs.py [--out-dir PATH] [--dry-run]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml


# --------------------------------------------------------------------------- #
# Per-battery-unit LUT voltages (mirrors the shipped config yamls)
# --------------------------------------------------------------------------- #

SLA_LUT_VOLTAGES = [11.6, 11.7, 11.9, 12.0, 12.2, 12.3, 12.4, 12.5,
                    12.55, 12.6, 12.65, 12.7]

HE26XX_LUT_VOLTAGES = [21.00, 21.84, 22.68, 23.52, 24.36, 25.20,
                       26.04, 26.88, 27.72, 28.56, 29.40]

RB20_LUT_VOLTAGES = [
    12.0, 12.2, 12.55, 12.7, 12.8, 12.88, 12.95, 13.0, 13.05, 13.08, 13.1,
    13.15, 13.18, 13.2, 13.22, 13.24, 13.26, 13.28, 13.3, 13.32, 13.35,
]

DTM8A31_LUT_VOLTAGES = [23.2, 23.4, 23.8, 24.0, 24.4, 24.6, 24.8,
                        25.0, 25.1, 25.2, 25.3, 25.4]


# Battery model -> per-unit LUT voltages.
BATTERY_LUTS: dict[str, list[float]] = {
    'he2613': HE26XX_LUT_VOLTAGES,
    'he2411': HE26XX_LUT_VOLTAGES,
    'he2410': HE26XX_LUT_VOLTAGES,
    'rb20': RB20_LUT_VOLTAGES,
    'es20_12c': SLA_LUT_VOLTAGES,
    'u1_35': SLA_LUT_VOLTAGES,
    'tlv1222': SLA_LUT_VOLTAGES,
    'dtm8a31': DTM8A31_LUT_VOLTAGES,
}


# --------------------------------------------------------------------------- #
# Sweep parameters
# --------------------------------------------------------------------------- #

OUT_OF_RANGE_MARGIN_V = 1.0   # Volts below LUT min / above LUT max for clamp tests.
CHARGER_ON_STEPS = 5          # Trailing samples with charger_connected=1 (test-side).
STEP_PERIOD_NS = 100_000_000  # 100 ms per step in the header timestamp.
START_SEC = 100


def build_voltage_sweep(per_unit_lut: list[float]) -> list[float]:
    """Build the per-unit voltage sweep: below-min + corners + midpoints + above-max."""
    corners_and_mids: list[float] = []
    for i, v in enumerate(per_unit_lut):
        corners_and_mids.append(v)
        if i + 1 < len(per_unit_lut):
            corners_and_mids.append((v + per_unit_lut[i + 1]) / 2.0)
    return (
        [per_unit_lut[0] - OUT_OF_RANGE_MARGIN_V]
        + corners_and_mids
        + [per_unit_lut[-1] + OUT_OF_RANGE_MARGIN_V]
    )


# --------------------------------------------------------------------------- #
# YAML serialization (compact list flow style, block maps)
# --------------------------------------------------------------------------- #

class _FloatList(list):
    """Marker class so PyYAML renders this list inline (flow style)."""


def _float_list_representer(dumper: yaml.Dumper, data: _FloatList):
    return dumper.represent_sequence(
        'tag:yaml.org,2002:seq', list(data), flow_style=True)


yaml.add_representer(_FloatList, _float_list_representer)


def write_fixture(path: Path, fixture: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w') as f:
        yaml.dump(fixture, f, sort_keys=False, default_flow_style=False)


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #

DEFAULT_OUT_REL = Path(
    'clearpath_hardware_interfaces/clearpath_hardware_interfaces/battery_state/'
    'test/fixtures/inputs'
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        '--out-dir', type=Path, default=None,
        help='Output directory. Defaults to <pkg>/.../battery_state/test/fixtures/inputs.',
    )
    parser.add_argument('--dry-run', action='store_true',
                        help='Print planned files without writing.')
    args = parser.parse_args()

    if args.out_dir is None:
        out_root = (Path(__file__).resolve().parent.parent / 'fixtures' / 'inputs').resolve()
    else:
        out_root = args.out_dir.resolve()

    print(f'output directory: {out_root}')
    print()

    written = 0
    for battery_name, per_unit_lut in BATTERY_LUTS.items():
        sweep = build_voltage_sweep(per_unit_lut)
        fixture = {
            'voltage_sweep_per_unit': _FloatList(sweep),
            'charger_on_steps': CHARGER_ON_STEPS,
            'step_period_ns': STEP_PERIOD_NS,
            'start_sec': START_SEC,
        }
        out_path = out_root / f'{battery_name}.yaml'
        if args.dry_run:
            print(f'  [dry-run] {battery_name}: {len(sweep)} sweep samples')
        else:
            write_fixture(out_path, fixture)
            print(f'  wrote {out_path.relative_to(out_root)} ({len(sweep)} sweep samples)')
        written += 1

    print()
    print(f'Fixtures generated: {written}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
