#!/usr/bin/env python3

import argparse
from pathlib import Path
import sys

from proton_ros2_adaptor_generator.config import AdaptorConfig, PackageConfig
from proton_ros2_adaptor_generator.generator import (
    create_jinja_env,
    generate_message_adaptor,
    generate_plugins_xml,
)
import yaml


def generate(schemas_dir: Path, output_dir: Path, package_name: str, schema_name: str) -> int:
    if Path(schema_name).name != schema_name or not schema_name.endswith('.yaml'):
        print(f'Invalid schema selection: {schema_name}', file=sys.stderr)
        return 1
    schema_path = schemas_dir / schema_name
    if not schema_path.is_file():
        print(f'Selected schema not found: {schema_path}', file=sys.stderr)
        return 1

    errors = []
    try:
        schema = yaml.safe_load(schema_path.read_text())
        if not isinstance(schema, dict) or not schema.get('messages'):
            print(f'No adaptor bindings found in {schema_path}', file=sys.stderr)
            return 1
        if package_name not in schema.get('adaptor_packages', []):
            errors.append(f'adaptor_packages must list {package_name}')
        adaptors = AdaptorConfig.from_yaml(schema_path)
        errors.extend(adaptors.validate())
        if not errors:
            errors.extend(adaptors.resolve_signal_ids(schema_path))
            errors.extend(adaptors.resolve_signal_capacities(schema_path))
    except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError) as error:
        print(f'{schema_path.name}: {error}', file=sys.stderr)
        return 1
    if errors:
        print('\n'.join(f'{schema_path.name}: {error}' for error in errors), file=sys.stderr)
        return 1

    source_dir = output_dir / 'src'
    source_dir.mkdir(parents=True, exist_ok=True)
    for stale_source in source_dir.glob('*_adaptor.cpp'):
        stale_source.unlink()

    environment = create_jinja_env()
    package_config = PackageConfig(package_name=package_name)
    for binding in adaptors.messages:
        generate_message_adaptor(environment, binding, output_dir)
    generate_plugins_xml(environment, package_config, adaptors, output_dir)

    sources = sorted(source_dir.glob('*_adaptor.cpp'))
    source_lines = ['set(PROTON_ADAPTOR_SOURCES']
    source_lines.extend(f'  "{source.as_posix()}"' for source in sources)
    source_lines.append(')')
    (output_dir / 'adaptor_sources.cmake').write_text('\n'.join(source_lines) + '\n')
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--schemas', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--package-name', required=True)
    parser.add_argument('--schema', required=True)
    args = parser.parse_args()
    return generate(args.schemas, args.output, args.package_name, args.schema)


if __name__ == '__main__':
    sys.exit(main())
