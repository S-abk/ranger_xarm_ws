"""The driven-base semantic description is the shipped one plus exactly the planar virtual joint.

Checks the file the build generated (MOBILE_BASE_SRDF, set by CMakeLists.txt) against the
shipped ranger_xarm.srdf, and that the check itself rejects anything else.
"""
import importlib.util
import os
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
SHIPPED = (PKG / 'config' / 'ranger_xarm.srdf').read_text(encoding='utf-8')


def _generator():
    spec = importlib.util.spec_from_file_location('make_mobile_base_srdf',
                                                  PKG / 'scripts' / 'make_mobile_base_srdf.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


GEN = _generator()


def test_built_file_is_shipped_plus_exactly_the_virtual_joint():
    built = Path(os.environ['MOBILE_BASE_SRDF']).read_text(encoding='utf-8')
    assert GEN.check(SHIPPED, built) == []
    assert built == GEN.generate(SHIPPED)


def test_shipped_file_has_no_virtual_joint():
    assert '<virtual_joint' not in SHIPPED


def test_wheels_file_is_mobile_base_plus_exactly_the_wheel_pairs():
    mobile = Path(os.environ['MOBILE_BASE_SRDF']).read_text(encoding='utf-8')
    wheels = Path(os.environ['MOBILE_BASE_WHEELS_SRDF']).read_text(encoding='utf-8')
    assert GEN.check_wheels(mobile, wheels) == []
    assert wheels == GEN.generate_wheels(mobile)
    assert len(GEN.WHEEL_PAIRS) == 4
    assert not any('xarm' in a + b for a, b in GEN.WHEEL_PAIRS)


def test_wheels_check_rejects_an_arm_pair_or_a_missing_pair():
    mobile = GEN.generate(SHIPPED)
    wheels = GEN.generate_wheels(mobile)
    arm = wheels.replace('front_left_wheel_link', 'xarm_link1', 1)
    assert GEN.check_wheels(mobile, arm) != []
    assert GEN.check_wheels(mobile, mobile) != []


@pytest.mark.parametrize('mutate', [
    lambda s: s.replace('parent_frame="odom"', 'parent_frame="map"'),
    lambda s: s.replace('type="planar"', 'type="floating"'),
    lambda s: s.replace(GEN.VIRTUAL_JOINT, GEN.VIRTUAL_JOINT + '\n  ' + GEN.VIRTUAL_JOINT),
    lambda s: s.replace('<group name="xarm6">', '<group name="arm">'),
    lambda s: s + '\n<!-- extra -->',
])
def test_check_rejects_any_other_difference(mutate):
    assert GEN.check(SHIPPED, mutate(GEN.generate(SHIPPED))) != []
