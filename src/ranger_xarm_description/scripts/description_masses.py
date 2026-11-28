#!/usr/bin/env python3
"""Mass and centre of mass of the base, the structure on it, the arm, the gripper and the whole
robot, computed from an expanded description.

    description_masses.py [name:=value ...]        expand the installed description with these
                                                   xacro arguments and print the table
    description_masses.py --urdf FILE              the same for an already expanded URDF
    description_masses.py --check [...]            also exit 1 if the base or the structure
                                                   misses its target
    description_masses.py --links [...]            print every link with an inertial too

Groups: the base is ranger_cad_link with the wheel, knuckle and suspension links; the arm is the
xarm_link* links; the gripper is the other xarm_* links and the wrist camera; the structure is
everything else (custom_structure_link and the sensor mounts that carry their own inertial).
Every link is placed with its joints at zero. Positions are in base_link; "above floor" adds
the description's base_footprint -> base_link height. Inertia is about each group's own centre,
in base_link axes, as the URDF's tensor elements.

Starts nothing.
"""
import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

# The targets the description is meant to meet: the base's user manual (75 kg with its wheels,
# centre of gravity 0.214 m above the floor, 14.9 mm to the right as read from its drawing, in
# the middle fore and aft) and the structure's parts list (44.5 kg at (-0.185, 0, 0.306)).
TARGETS = {
    'base': {'mass': 75.0, 'mass_tol': 0.1, 'com': (0.0, -0.0149, -0.113), 'com_tol': 0.002},
    'structure': {'mass': 44.5, 'mass_tol': 0.1, 'com': (-0.185, 0.0, 0.306), 'com_tol': 0.005},
}
GROUPS = ('base', 'structure', 'arm', 'gripper')


def group_of(link):
    if link == 'ranger_cad_link' or link.endswith(('_steer_link', '_wheel_link', '_suspension_link')):
        return 'base'
    if link.startswith('xarm_link'):
        return 'arm'
    if link.startswith(('xarm_', 'wrist_d435')):
        return 'gripper'
    return 'structure'


def expand(mappings=None):
    """The installed description expanded with these xacro arguments, as an element tree."""
    import xacro
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory('ranger_xarm_description')) / 'urdf' / 'ranger_xarm.urdf.xacro'
    return ET.fromstring(xacro.process_file(str(path), mappings=dict(mappings or {})).toxml())


def _rot(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def _tf(origin):
    T = np.eye(4)
    if origin is not None:
        T[:3, :3] = _rot(*[float(v) for v in origin.get('rpy', '0 0 0').split()])
        T[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
    return T


def inertials(root):
    """{link: (group, mass, centre in base_link, inertia about the centre in base_link axes)}."""
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    cache = {}

    def pose(link):
        if link not in cache:
            j = joints.get(link)
            cache[link] = np.eye(4) if j is None else pose(j.find('parent').get('link')) @ _tf(j.find('origin'))
        return cache[link]

    to_base = np.linalg.inv(pose('base_link'))
    out = {}
    for link in root.findall('link'):
        i = link.find('inertial')
        if i is None:
            continue
        name = link.get('name')
        T = to_base @ pose(name) @ _tf(i.find('origin'))
        a = {k: float(i.find('inertia').get(k)) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')}
        I = np.array([[a['ixx'], a['ixy'], a['ixz']], [a['ixy'], a['iyy'], a['iyz']],
                      [a['ixz'], a['iyz'], a['izz']]])
        out[name] = (group_of(name), float(i.find('mass').get('value')), T[:3, 3].copy(),
                     T[:3, :3] @ I @ T[:3, :3].T)
    return out


def floor_height(root):
    """base_link's height above base_footprint, as the description states it."""
    j = next(j for j in root.findall('joint') if j.find('child').get('link') == 'base_link')
    return float(j.find('origin').get('xyz').split()[2])


def combine(items):
    """(mass, centre, inertia about the centre) of a list of such triples; None if empty."""
    items = list(items)
    if not items:
        return None
    M = sum(m for m, _, _ in items)
    C = sum(m * c for m, c, _ in items) / M
    I = sum(Ic + m * (np.dot(c - C, c - C) * np.eye(3) - np.outer(c - C, c - C)) for m, c, Ic in items)
    return M, C, I


def groups(links):
    """{group or 'robot': (mass, centre, inertia) or None}."""
    out = {g: combine((m, c, I) for gg, m, c, I in links.values() if gg == g) for g in GROUPS}
    out['robot'] = combine((m, c, I) for _, m, c, I in links.values())
    return out


def misses(totals):
    """The targets a set of group totals misses, as readable lines (empty if none)."""
    bad = []
    for g, t in TARGETS.items():
        got = totals.get(g)
        if got is None:
            bad.append(f'{g}: no links')
            continue
        M, C, _ = got
        if abs(M - t['mass']) > t['mass_tol']:
            bad.append(f"{g}: {M:.3f} kg, target {t['mass']} +- {t['mass_tol']}")
        d = float(np.linalg.norm(C - np.array(t['com'])))
        if d > t['com_tol']:
            bad.append(f"{g}: centre {np.round(C, 4).tolist()} is {d * 1000:.1f} mm from "
                       f"{list(t['com'])}, tolerance {t['com_tol'] * 1000:.0f} mm")
    return bad


def table(links, floor, show_links=False):
    f = lambda c: '(' + ', '.join(f'{v:+.4f}' for v in c) + ')'  # noqa: E731
    lines = []
    if show_links:
        lines.append('| link | group | kg | centre (m, base_link) |')
        lines.append('|---|---|---|---|')
        for n, (g, m, c, _) in sorted(links.items(), key=lambda kv: (kv[1][0], kv[0])):
            lines.append(f'| {n} | {g} | {m:.4f} | {f(c)} |')
        lines.append('')
    lines.append('| group | kg | centre (m, base_link) | above floor (m) | ixx | iyy | izz | ixz |')
    lines.append('|---|---|---|---|---|---|---|---|')
    for g, v in groups(links).items():
        if v is None:
            lines.append(f'| {g} | 0 | | | | | | |')
            continue
        M, C, I = v
        lines.append(f'| {g} | {M:.3f} | {f(C)} | {C[2] + floor:.4f} | {I[0, 0]:.3f} | {I[1, 1]:.3f} '
                     f'| {I[2, 2]:.3f} | {I[0, 2]:+.3f} |')
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--urdf', help='an expanded URDF instead of expanding the installed xacro')
    ap.add_argument('--check', action='store_true', help='exit 1 if a target is missed')
    ap.add_argument('--links', action='store_true', help='print every link with an inertial')
    ap.add_argument('args', nargs='*', help='xacro arguments, name:=value')
    a = ap.parse_args(argv)
    if a.urdf:
        if a.args:
            ap.error('xacro arguments and --urdf do not go together')
        root = ET.parse(a.urdf).getroot()
    else:
        root = expand(dict(s.split(':=', 1) for s in a.args))
    links = inertials(root)
    print(table(links, floor_height(root), a.links))
    if a.check:
        bad = misses(groups(links))
        for line in bad:
            print('MISS ' + line)
        print('targets: ' + ('missed' if bad else 'met'))
        return 1 if bad else 0
    return 0


if __name__ == '__main__':
    sys.exit(main())
