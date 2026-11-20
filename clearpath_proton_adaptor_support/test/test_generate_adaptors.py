import importlib.util
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

DRIVER = Path(__file__).resolve().parents[1] / 'scripts' / 'generate_adaptors.py'
SPEC = importlib.util.spec_from_file_location('generate_adaptors', DRIVER)
GENERATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GENERATOR)
PLATFORMS = {
    'core.yaml': 'core',
    'd1x0.yaml': 'dingo',
    'j100.yaml': 'jackal',
    'r100.yaml': 'ridgeback',
    'w200.yaml': 'warthog',
}


@pytest.fixture
def schemas(tmp_path):
    selected = {
        'adaptor_packages': ['test_adaptors'],
        'signals': [{'name': 'value', 'id': 42, 'type': 'uint32'}],
        'messages': [{
            'name': 'Selected',
            'ros2_type': 'std_msgs/msg/UInt32',
            'mapping': [{
                'ros2.path': 'data', 'proton.signal': 'value', 'type': 'uint32',
            }],
        }],
    }
    (tmp_path / 'selected.yaml').write_text(yaml.safe_dump(selected))
    (tmp_path / 'unselected.yaml').write_text('invalid: [')
    return tmp_path


def test_selection_precedes_parsing_and_cleans_stale_sources(schemas):
    output = schemas / 'output'
    (output / 'src').mkdir(parents=True)
    (output / 'src' / 'unselected_adaptor.cpp').write_text('stale')
    assert GENERATOR.generate(schemas, output, 'test_adaptors', 'selected.yaml') == 0
    assert {source.name for source in (output / 'src').glob('*.cpp')} == {
        'selected_adaptor.cpp',
    }
    assert '42' in (output / 'src' / 'selected_adaptor.cpp').read_text()
    plugins = ET.parse(output / 'plugins.xml').getroot()
    assert plugins.attrib['path'] == 'test_adaptors'
    assert [entry.attrib['type'] for entry in plugins.findall('class')] == [
        'proton_ros2_adaptors::SelectedAdaptor',
    ]


@pytest.mark.parametrize(
    'selection', ['missing.yaml', '../selected.yaml', '/selected.yaml', '']
)
def test_invalid_selection_writes_nothing(schemas, selection):
    output = schemas / 'output'
    assert GENERATOR.generate(schemas, output, 'test_adaptors', selection) == 1
    assert not output.exists()


@pytest.mark.parametrize('problem', [
    'owner', 'duplicate', 'signal', 'capacity', 'path', 'index', 'empty', 'malformed',
])
def test_selected_schema_validation_is_retained(schemas, problem):
    schema_path = schemas / 'selected.yaml'
    schema = yaml.safe_load(schema_path.read_text())
    mapping = schema['messages'][0]['mapping'][0]
    if problem == 'owner':
        schema['adaptor_packages'] = ['wrong_package']
    elif problem == 'duplicate':
        schema['messages'].append(schema['messages'][0])
    elif problem == 'signal':
        mapping['proton.signal'] = 'missing'
    elif problem == 'capacity':
        mapping.update({'type': 'bytes', 'proton.index': 2})
        schema['signals'][0].update({'type': 'bytes', 'capacity': 2})
    elif problem == 'path':
        mapping['ros2.path'] = 'invalid[-1]'
    elif problem == 'index':
        mapping.update({'type': 'bytes', 'proton.index': 'bad'})
    elif problem == 'empty':
        schema['messages'] = []
    schema_path.write_text(
        'invalid: [' if problem == 'malformed' else yaml.safe_dump(schema)
    )
    output = schemas / 'output'
    assert GENERATOR.generate(schemas, output, 'test_adaptors', 'selected.yaml') == 1
    assert not output.exists()


@pytest.mark.parametrize('schema_name,platform', PLATFORMS.items())
def test_installed_platform_plugins_have_no_cross_platform_bindings(
    tmp_path, schema_name, platform
):
    if 'CLEARPATH_TEST_SCHEMA_DIR' in os.environ:
        schema_dir = Path(os.environ['CLEARPATH_TEST_SCHEMA_DIR'])
    else:
        schema_dir = Path(get_package_share_directory('clearpath_firmware')) / 'proton'
    schema = yaml.safe_load((schema_dir / schema_name).read_text())
    package = f'clearpath_proton_{platform}_adaptors'
    assert schema['adaptor_packages'] == [package]
    assert GENERATOR.generate(schema_dir, tmp_path, package, schema_name) == 0
    bindings = {entry['name'] for entry in schema['messages']}
    plugins = ET.parse(tmp_path / 'plugins.xml').getroot()
    assert plugins.attrib['path'] == package
    assert {entry.attrib['type'] for entry in plugins.findall('class')} == {
        f'proton_ros2_adaptors::{binding}Adaptor' for binding in bindings
    }
    assert len(list((tmp_path / 'src').glob('*_adaptor.cpp'))) == len(bindings)
    source_text = '\n'.join(
        source.read_text() for source in (tmp_path / 'src').glob('*.cpp')
    )
    for other_name in PLATFORMS:
        if other_name == schema_name:
            continue
        other = yaml.safe_load((schema_dir / other_name).read_text())
        for entry in other['messages']:
            assert f'"{entry["name"]}"' not in source_text
