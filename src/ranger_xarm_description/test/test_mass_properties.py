"""The description's masses come from config/mass_properties.yaml, and match the targets.

- The committed urdf/mass_properties.generated.xacro is exactly what the data gives, and the
  generator's check rejects a stale file.
- In each variant below (no wheels, rigid wheels, wheels on suspension, no arm, the driven
  simulator with sensors), computed by scripts/description_masses.py from the expanded
  description: the base weighs 75.0 kg (within 0.1) with its centre within 2 mm of the manual's;
  the structure (everything fixed to the base that is not the base, the arm or the gripper)
  weighs 44.5 kg (within 0.1) with its centre within 5 mm of the parts list's; nothing is
  counted twice; and the base has the same mass properties in every variant.
- The data's geometry (floor height, wheel stations, the parts other links carry) is the
  description's, and the suspension's rest settings are the data's.

Expands the installed description (xacro, $(find ...)); starts nothing.
"""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

PKG = Path(__file__).resolve().parents[1]
SIM_DRIVE_SENSORS = {
    'xarm_ros2_control_plugin': 'gz_ros2_control/GazeboSimSystem', 'xarm_load_gazebo_plugin': 'true',
    'add_gripper': 'true', 'use_wheels': 'true', 'fix_base_to_world': 'false', 'gz_sensors': 'true',
    'wheels_command_interface': 'velocity'}
# name: (xacro arguments, wheels modelled, suspension modelled)
VARIANTS = {
    'no wheels': ({}, False, False),
    'rigid wheels': ({'use_wheels': 'true', 'use_suspension': 'false'}, True, False),
    'wheels on suspension': ({'use_wheels': 'true', 'use_suspension': 'true'}, True, True),
    'no arm': ({'use_xarm': 'false'}, False, False),
    'driven simulator with sensors': (SIM_DRIVE_SENSORS, True, True),
}


def _load(name):
    spec = importlib.util.spec_from_file_location(name, PKG / 'scripts' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


GEN = _load('make_mass_properties')
DM = _load('description_masses')
DATA = GEN.load()


@pytest.fixture(scope='module', params=list(VARIANTS))
def variant(request):
    args, wheels, suspension = VARIANTS[request.param]
    root = DM.expand(args)
    return request.param, root, DM.inertials(root), wheels, suspension


@pytest.fixture(scope='module')
def all_variants():
    return {k: DM.inertials(DM.expand(v[0])) for k, v in VARIANTS.items()}


# ---- the generated file --------------------------------------------------------------------
def test_generated_file_is_what_the_data_gives():
    assert GEN.OUT.read_text(encoding='utf-8') == GEN.generate(DATA)
    assert GEN.main(['--check']) == 0


def test_check_rejects_a_stale_file(tmp_path, monkeypatch):
    stale = GEN.generate(DATA).replace('<mass value="44.414"/>', '<mass value="44.5"/>')
    assert stale != GEN.generate(DATA)
    out = tmp_path / 'mass_properties.generated.xacro'
    out.write_text(stale, encoding='utf-8')
    monkeypatch.setattr(GEN, 'OUT', out)
    assert GEN.main(['--check']) == 1


@pytest.mark.parametrize('change', [
    lambda d: d['structure']['parts'][0].update(kg=d['structure']['parts'][0]['kg'] + 0.01),
    lambda d: d['base'].update(lateral_side='left'),
    lambda d: d['suspension_rest'].update(rear=d['suspension_rest']['rear'] + 0.0001),
    lambda d: d['corners']['wheel'].update(mass=3.1),
])
def test_any_change_to_the_data_changes_the_generated_file(change):
    d = GEN.load()
    change(d)
    assert GEN.generate(d) != GEN.generate(DATA)


def test_the_side_of_the_base_offset_is_a_data_value():
    d = GEN.load()
    assert d['base']['lateral_side'] in ('right', 'left')
    right = GEN.base_com(d)[1]
    d['base']['lateral_side'] = 'left'
    assert GEN.base_com(d)[1] == pytest.approx(-right)


# ---- INV-2 in every variant ---------------------------------------------------------------
def test_targets_met(variant):
    name, _, links, _, _ = variant
    assert DM.misses(DM.groups(links)) == [], name


def test_the_targets_are_the_datas():
    """The script's targets and the data agree, so the data cannot drift from them unseen."""
    assert DM.TARGETS['base']['mass'] == DATA['base']['mass']
    assert np.allclose(DM.TARGETS['base']['com'], GEN.base_com(DATA), atol=1e-4)
    parts = GEN.structure(DATA)[0]
    assert parts[0] == pytest.approx(DM.TARGETS['structure']['mass'], abs=0.1)
    assert np.linalg.norm(np.array(parts[1]) - DM.TARGETS['structure']['com']) <= 0.001


def test_nothing_is_counted_twice(variant):
    """The structure's links sum to the parts list exactly: custom_structure_link carries the
    parts less what the other structure links carry, and no other link carries mass."""
    name, _, links, _, _ = variant
    M, C, I = DM.groups(links)['structure']
    Mp, Cp, Ip = GEN.structure(DATA)[0]
    assert M == pytest.approx(Mp, abs=1e-3), name
    assert np.allclose(C, Cp, atol=1e-5), name
    assert np.allclose(I, np.array(Ip), atol=1e-4), name
    others = {n for n, (g, *_rest) in links.items() if g == 'structure'} - {'custom_structure_link'}
    assert others == {e['link'] for e in DATA['structure']['carried_elsewhere']}, name


def test_the_base_is_the_same_in_every_variant(all_variants):
    ref = DM.groups(all_variants['wheels on suspension'])['base']
    for name, links in all_variants.items():
        M, C, I = DM.groups(links)['base']
        assert M == pytest.approx(ref[0], abs=1e-3), name
        assert np.allclose(C, ref[1], atol=1e-5), name
        assert np.allclose(I, ref[2], atol=1e-4), name


# ---- the data's geometry is the description's ---------------------------------------------
def test_floor_height_is_the_descriptions(variant):
    _, root, _, _, _ = variant
    assert DM.floor_height(root) == pytest.approx(DATA['floor_to_base_link'], abs=1e-9)


def test_corner_parts_are_where_the_data_puts_them(variant):
    name, _, links, wheels, suspension = variant
    if not wheels:
        assert not any(n.endswith(('_wheel_link', '_steer_link', '_suspension_link')) for n in links)
        return
    corners = GEN.corner_items(DATA, suspension=suspension)
    for part, suffix in (('wheel', '_wheel_link'), ('knuckle', '_steer_link'),
                         ('suspension_link', '_suspension_link')):
        expected = sorted((round(c[0], 6), round(c[1], 6), round(c[2], 6), m) for m, c, _ in corners[part])
        got = sorted((round(c[0], 6), round(c[1], 6), round(c[2], 6), m)
                     for n, (_, m, c, _) in links.items() if n.endswith(suffix))
        assert got == expected, (name, part)
        for n in (n for n in links if n.endswith(suffix)):
            assert np.allclose(links[n][3], np.array(corners[part][0][2]), atol=1e-9), (name, n)


def test_parts_carried_elsewhere_are_the_descriptions(variant):
    name, _, links, _, _ = variant
    for e in DATA['structure']['carried_elsewhere']:
        _, m, c, I = links[e['link']]
        assert m == pytest.approx(e['mass'], abs=1e-9), name
        assert np.allclose(c, e['com'], atol=1e-6), name
        assert np.allclose(I, np.diag(e['inertia']), atol=1e-9), name


def test_rest_settings_are_the_datas():
    root = DM.expand(VARIANTS['wheels on suspension'][0])
    refs = {g.get('reference'): float(g.find('springReference').text)
            for g in root.findall('gazebo') if g.find('springReference') is not None}
    rest = DATA['suspension_rest']
    assert refs == {
        'front_left_suspension_joint': -rest['front'], 'front_right_suspension_joint': -rest['front'],
        'rear_left_suspension_joint': -rest['rear'], 'rear_right_suspension_joint': -rest['rear']}
