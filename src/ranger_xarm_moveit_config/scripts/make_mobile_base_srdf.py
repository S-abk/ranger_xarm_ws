#!/usr/bin/env python3
"""Generate the semantic description used while the base is driven.

    make_mobile_base_srdf.py <shipped ranger_xarm.srdf> <output>
    make_mobile_base_srdf.py --check <shipped> <generated>
    make_mobile_base_srdf.py --wheels <mobile-base srdf> <output>
    make_mobile_base_srdf.py --check-wheels <mobile-base srdf> <generated>

The output is the shipped semantic description with exactly one element added: a planar
virtual joint from odom to base_footprint, the description's root link when the base is not
welded to the world (fix_base_to_world:=false). MoveIt then plans in odom and takes the base's
pose from TF, so the lidar obstacle map, which move_group keeps in the planning frame, stays put
in the world while the base drives. Nothing else differs; --check proves that.

--wheels makes a third file, for descriptions that have wheel links (use_wheels:=true, the
simulators with drive_base:=true): the mobile-base file plus exactly the wheel-against-body
pairs below, for which a start-state contact was observed in simulation (cause not
established). No pair names an arm link. The mobile-base
file is not changed: it also serves the real robot, whose description has no wheel links.

Never edit the generated files: they are rebuilt from the shipped one on every build.
"""
import sys
import xml.etree.ElementTree as ET

VIRTUAL_JOINT = ('<virtual_joint name="virtual_joint" type="planar" '
                 'parent_frame="odom" child_link="base_footprint"/>')


WHEEL_PAIRS = [(w, 'ranger_cad_link') for w in ('front_left_wheel_link', 'front_right_wheel_link',
                                                  'rear_left_wheel_link', 'rear_right_wheel_link')]
WHEEL_REASON = 'Not adjacent; start-state contact observed in simulation'


def _pair_element(link1, link2):
    return f'<disable_collisions link1="{link1}" link2="{link2}" reason="{WHEEL_REASON}"/>'


def generate_wheels(mobile: str) -> str:
    end = mobile.rindex('</robot>')
    lines = ''.join(f'  {_pair_element(a, b)}\n' for a, b in WHEEL_PAIRS)
    return mobile[:end] + lines + mobile[end:]


def check_wheels(mobile: str, generated: str) -> list:
    """Return the problems; empty when generated is mobile plus exactly the wheel pairs."""
    problems = []
    lines = ''.join(f'  {_pair_element(a, b)}\n' for a, b in WHEEL_PAIRS)
    if generated.count(lines) != 1 or generated.replace(lines, '', 1) != mobile:
        problems.append('removing the wheel pairs does not give the mobile-base file byte for byte')
    a, b = ET.fromstring(mobile), ET.fromstring(generated)
    for e in list(a) + list(b):
        e.tail = None   # whitespace between elements is covered by the byte check above
    before = [ET.tostring(e) for e in a]
    after = [ET.tostring(e) for e in b]
    added = [e for e in list(b)[len(before):]]
    if after[:len(before)] != before or a.attrib != b.attrib:
        problems.append('the mobile-base elements are not kept unchanged and in order')
    got = [(e.tag, e.get('link1'), e.get('link2')) for e in added]
    if got != [('disable_collisions', x, y) for x, y in WHEEL_PAIRS]:
        problems.append(f'the added elements are not exactly the wheel pairs: {got}')
    if any('xarm' in (e.get('link1') or '') + (e.get('link2') or '') for e in added):
        problems.append('a pair names an arm link')
    return problems


def generate(shipped: str) -> str:
    start = shipped.index('<robot')
    end = shipped.index('>', start) + 1
    return shipped[:end] + '\n  ' + VIRTUAL_JOINT + shipped[end:]


def check(shipped: str, generated: str) -> list:
    """Return the problems; empty when generated is shipped plus exactly the virtual joint."""
    problems = []
    if generated.count(VIRTUAL_JOINT) != 1:
        problems.append('the virtual joint is not present exactly once')
    if generated.replace('\n  ' + VIRTUAL_JOINT, '', 1) != shipped:
        problems.append('removing the virtual joint does not give the shipped file byte for byte')
    a, b = ET.fromstring(shipped), ET.fromstring(generated)
    extra = [e for e in b if e.tag == 'virtual_joint']
    rest = [ET.tostring(e) for e in b if e.tag != 'virtual_joint']
    if len(extra) != 1 or extra[0].attrib != {'name': 'virtual_joint', 'type': 'planar',
                                              'parent_frame': 'odom', 'child_link': 'base_footprint'}:
        problems.append('the virtual joint element is not the planar odom -> base_footprint one')
    if rest != [ET.tostring(e) for e in a] or a.attrib != b.attrib:
        problems.append('the other elements differ from the shipped file')
    if any(e.tag == 'virtual_joint' for e in a):
        problems.append('the shipped file already has a virtual joint')
    return problems


def main(argv):
    if len(argv) == 4 and argv[1] in ('--wheels', '--check-wheels'):
        with open(argv[2], encoding='utf-8') as f:
            mobile = f.read()
        if argv[1] == '--wheels':
            generated = generate_wheels(mobile)
            problems = check_wheels(mobile, generated)
            if not problems:
                with open(argv[3], 'w', encoding='utf-8') as f:
                    f.write(generated)
        else:
            with open(argv[3], encoding='utf-8') as f:
                problems = check_wheels(mobile, f.read())
            print('\n'.join(f'FAIL {p}' for p in problems) or
                  'OK wheels file = mobile-base + exactly the wheel-against-body pairs')
        if problems and argv[1] == '--wheels':
            print('\n'.join(problems), file=sys.stderr)
        return 1 if problems else 0
    if len(argv) == 4 and argv[1] == '--check':
        with open(argv[2], encoding='utf-8') as f:
            shipped = f.read()
        with open(argv[3], encoding='utf-8') as f:
            generated = f.read()
        problems = check(shipped, generated)
        for p in problems:
            print(f'FAIL {p}')
        if not problems:
            print('OK generated semantic description = shipped + exactly the planar virtual joint')
        return 1 if problems else 0
    if len(argv) != 3:
        print(__doc__)
        return 2
    with open(argv[1], encoding='utf-8') as f:
        shipped = f.read()
    generated = generate(shipped)
    problems = check(shipped, generated)
    if problems:
        print('\n'.join(problems), file=sys.stderr)
        return 1
    with open(argv[2], 'w', encoding='utf-8') as f:
        f.write(generated)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
