#!/usr/bin/env python3
"""Generate the semantic description used while the base is driven.

    make_mobile_base_srdf.py <shipped ranger_xarm.srdf> <output>
    make_mobile_base_srdf.py --check <shipped> <generated>

The output is the shipped semantic description with exactly one element added: a planar
virtual joint from odom to base_footprint, the description's root link when the base is not
welded to the world (fix_base_to_world:=false). MoveIt then plans in odom and takes the base's
pose from TF, so the lidar obstacle map, which move_group keeps in the planning frame, stays put
in the world while the base drives. Nothing else differs; --check proves that.

Never edit the generated file: it is rebuilt from the shipped one on every build.
"""
import sys
import xml.etree.ElementTree as ET

VIRTUAL_JOINT = ('<virtual_joint name="virtual_joint" type="planar" '
                 'parent_frame="odom" child_link="base_footprint"/>')


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
