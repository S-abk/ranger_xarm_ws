#!/usr/bin/env python3
"""Write urdf/mass_properties.generated.xacro from config/mass_properties.yaml.

    make_mass_properties.py            write the generated file
    make_mass_properties.py --check    exit 1 if the generated file is not what the data gives
    make_mass_properties.py --report   print the base, the structure and each link's inertial

The generated file holds, for the description to use:
  - the macro ranger_cad_inertial (wheels, suspension): the base body's <inertial> in each of
    its three variants. Without modelled wheels the body carries the whole base (75 kg, corners
    included); with wheels it carries the rest. The base as a whole has the same mass, centre
    and inertia in every variant.
  - the macro custom_structure_inertial: the structure parts, less what other links carry.
  - properties for the corner links' masses and inertias and the per-axle rest settings.

Method: boxes and bars as solid boxes, the rest as point masses, summed about the composite
centre with the parallel-axis theorem. Inertia in the URDF convention (tensor elements:
ixz = -sum m dx dz).
"""
import argparse
import sys
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]
DATA = PKG / 'config' / 'mass_properties.yaml'
OUT = PKG / 'urdf' / 'mass_properties.generated.xacro'
CORNERS = {'front_left': (1, 1), 'front_right': (1, -1), 'rear_left': (-1, 1), 'rear_right': (-1, -1)}


# ---- small 3-vector / 3x3 helpers (no numpy needed) ----------------------------------------
def zeros():
    return [[0.0] * 3 for _ in range(3)]


def diag(a, b, c):
    return [[a, 0.0, 0.0], [0.0, b, 0.0], [0.0, 0.0, c]]


def add(A, B, s=1.0):
    return [[A[i][j] + s * B[i][j] for j in range(3)] for i in range(3)]


def shift(m, d):
    """Parallel-axis term m (|d|^2 E - d d^T)."""
    dd = sum(v * v for v in d)
    return [[m * ((dd if i == j else 0.0) - d[i] * d[j]) for j in range(3)] for i in range(3)]


def box(m, dx, dy, dz):
    return diag(m * (dy * dy + dz * dz) / 12, m * (dx * dx + dz * dz) / 12, m * (dx * dx + dy * dy) / 12)


def combine(items):
    """items: (m, c, I about c) -> (M, C, I about C)."""
    M = sum(m for m, _, _ in items)
    C = [sum(m * c[k] for m, c, _ in items) / M for k in range(3)]
    I = zeros()
    for m, c, Ic in items:
        I = add(add(I, Ic), shift(m, [c[k] - C[k] for k in range(3)]))
    return M, C, I


def remove(total, items):
    """total (M, C, I about C) minus items -> what is left, about its own centre."""
    M, C, I = total
    for m, c, Ic in items:
        I = add(add(I, Ic, -1.0), shift(m, [c[k] - C[k] for k in range(3)]), -1.0)
    Mr = M - sum(m for m, _, _ in items)
    Cr = [(M * C[k] - sum(m * c[k] for m, c, _ in items)) / Mr for k in range(3)]
    return Mr, Cr, add(I, shift(Mr, [Cr[k] - C[k] for k in range(3)]), -1.0)


# ---- the model ---------------------------------------------------------------------------
def load(path=DATA):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def base_com(d):
    b = d['base']
    side = {'right': -1.0, 'left': 1.0}[b['lateral_side']]
    return [b['com_x'], side * b['lateral_offset'], b['com_z_above_floor'] - d['floor_to_base_link']]


def corner_items(d, suspension=True):
    """{part: [(m, c, I)] over the four corners} at the joints' zero."""
    c = d['corners']
    w, k, s = c['wheel'], c['knuckle'], c['suspension_link']
    r, h = w['radius'], w['width']
    I_wheel = diag(w['mass'] * (3 * r * r + h * h) / 12, w['mass'] * r * r / 2,
                   w['mass'] * (3 * r * r + h * h) / 12)
    out = {'wheel': [], 'knuckle': [], 'suspension_link': []}
    for sx, sy in CORNERS.values():
        axle = [sx * c['half_wheelbase'], sy * c['half_track'], c['axle_z']]
        out['wheel'].append((w['mass'], axle, I_wheel))
        out['knuckle'].append((k['mass'], [axle[0], axle[1], axle[2] + k['com_z']], diag(*k['inertia'])))
        if suspension:
            out['suspension_link'].append((s['mass'], axle, diag(*s['inertia'])))
    return out


def base_variants(d):
    """The body link's (m, c, I) per variant, and the whole base (identical in every variant)."""
    b = d['base']
    C = base_com(d)
    corners = corner_items(d)
    all_corners = corners['wheel'] + corners['knuckle'] + corners['suspension_link']
    m_corners = sum(m for m, _, _ in all_corners)
    m_body = b['mass'] - m_corners
    c_body = [(b['mass'] * C[k] - sum(m * c[k] for m, c, _ in all_corners)) / m_body for k in range(3)]
    body_susp = (m_body, c_body, box(m_body, *b['body_box']))
    whole = combine([body_susp] + all_corners)
    rigid = remove(whole, corners['wheel'] + corners['knuckle'])
    return {'none': whole, 'rigid': rigid, 'suspension': body_susp}, whole


def structure_parts(d, masses=None):
    floor = d['floor_to_base_link']
    items = []
    for i, p in enumerate(d['structure']['parts']):
        m = masses[i] if masses else p['kg']
        c = [p['x'], 0.0, p['z_above_floor'] - floor]
        if p['shape'] == 'box':
            items.append((m, c, box(m, *p['dims'])))
        elif p['shape'] == 'pair':
            dx, dy, dz, y = p['dims']
            for s in (1, -1):
                items.append((m / 2, [c[0], s * y, c[2]], box(m / 2, dx, dy, dz)))
        elif p['shape'] == 'point':
            items.append((m, c, zeros()))
        else:
            raise ValueError(f"unknown shape {p['shape']!r} for {p['name']!r}")
    return items


def structure(d):
    """(parts composite, custom_structure_link's share, the items carried elsewhere)."""
    parts = combine(structure_parts(d))
    elsewhere = [(e['mass'], e['com'], diag(*e['inertia'])) for e in d['structure']['carried_elsewhere']]
    return parts, remove(parts, elsewhere), elsewhere


# ---- output ------------------------------------------------------------------------------
def num(v, places=6):
    s = f'{v:.{places}f}'
    return '0' if float(s) == 0.0 else s.rstrip('0').rstrip('.') if '.' in s else s


def inertial(m, c, I, indent, note=None):
    p = ' ' * indent
    lines = [f'{p}<inertial>']
    if note:
        lines.append(f'{p}  <!-- {note} -->')
    lines += [f'{p}  <origin xyz="{num(c[0])} {num(c[1])} {num(c[2])}" rpy="0 0 0"/>',
              f'{p}  <mass value="{num(m, 4)}"/>',
              f'{p}  <inertia ixx="{num(I[0][0])}" ixy="{num(I[0][1])}" ixz="{num(I[0][2])}" '
              f'iyy="{num(I[1][1])}" iyz="{num(I[1][2])}" izz="{num(I[2][2])}"/>',
              f'{p}</inertial>']
    return '\n'.join(lines)


def generate(d):
    variants, _ = base_variants(d)
    _, link, _ = structure(d)
    b, c = d['base'], d['corners']
    side_note = (f"base centre y: {b['lateral_offset']} m to the {b['lateral_side']}, "
                 f"{b['lateral_basis']}")
    note = {
        'none': f"the whole base, corners included: {b['mass']} kg ({b['mass_basis']}); {side_note}",
        'rigid': f"the base less its wheels and knuckles; {side_note}",
        'suspension': f"the base less its wheels, knuckles and suspension links; {side_note}",
    }
    prop = lambda n, v: f'  <xacro:property name="{n}" value="{v!r}"/>'  # noqa: E731
    out = [
        '<?xml version="1.0"?>',
        '<!-- GENERATED by scripts/make_mass_properties.py from config/mass_properties.yaml.',
        '     Do not edit: change the data file and run the script; the package test fails if',
        '     this file is not what the data gives. -->',
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">',
        '  <!-- ranger_cad_link. wheels / suspension: as passed to ranger_static_geometry. -->',
        '  <xacro:macro name="ranger_cad_inertial" params="wheels suspension">',
        '    <xacro:unless value="${wheels}">',
        inertial(*variants['none'], 6, note['none']),
        '    </xacro:unless>',
        '    <xacro:if value="${wheels}">',
        '      <xacro:unless value="${suspension}">',
        inertial(*variants['rigid'], 8, note['rigid']),
        '      </xacro:unless>',
        '      <xacro:if value="${suspension}">',
        inertial(*variants['suspension'], 8, note['suspension']),
        '      </xacro:if>',
        '    </xacro:if>',
        '  </xacro:macro>',
        '',
        '  <!-- custom_structure_link: the structure parts less what other links carry. -->',
        '  <xacro:macro name="custom_structure_inertial" params="">',
        inertial(*link, 4, 'structure parts (config/mass_properties.yaml), less '
                 + ', '.join(e['link'] for e in d['structure']['carried_elsewhere'])),
        '  </xacro:macro>',
        '',
        '  <!-- Wheel corners (ranger_wheels.xacro). -->',
        prop('mp_wheel_mass', c['wheel']['mass']),
        prop('mp_knuckle_mass', c['knuckle']['mass']),
        prop('mp_knuckle_com_z', c['knuckle']['com_z']),
        prop('mp_knuckle_ixx', c['knuckle']['inertia'][0]),
        prop('mp_knuckle_iyy', c['knuckle']['inertia'][1]),
        prop('mp_knuckle_izz', c['knuckle']['inertia'][2]),
        prop('mp_suspension_link_mass', c['suspension_link']['mass']),
        prop('mp_suspension_link_ixx', c['suspension_link']['inertia'][0]),
        prop('mp_suspension_link_iyy', c['suspension_link']['inertia'][1]),
        prop('mp_suspension_link_izz', c['suspension_link']['inertia'][2]),
        '  <!-- Suspension rest settings per axle: springReference = -setting. -->',
        prop('mp_suspension_rest_front', d['suspension_rest']['front']),
        prop('mp_suspension_rest_rear', d['suspension_rest']['rear']),
        '</robot>',
        '',
    ]
    return '\n'.join(out)


def report(d):
    variants, whole = base_variants(d)
    parts, link, elsewhere = structure(d)
    f = lambda c: '(' + ', '.join(f'{v:+.4f}' for v in c) + ')'  # noqa: E731
    i = lambda I: ' '.join(f'{k} {I[a][b]:+.4f}' for k, a, b in  # noqa: E731
                           (('ixx', 0, 0), ('iyy', 1, 1), ('izz', 2, 2), ('ixy', 0, 1), ('ixz', 0, 2), ('iyz', 1, 2)))
    print(f'base, whole: {whole[0]:.3f} kg at {f(whole[1])}; {i(whole[2])}')
    for k, v in variants.items():
        print(f'  ranger_cad_link ({k}): {v[0]:.3f} kg at {f(v[1])}; {i(v[2])}')
    print(f'structure parts: {parts[0]:.3f} kg at {f(parts[1])}; {i(parts[2])}')
    for m, c, _ in elsewhere:
        print(f'  carried elsewhere: {m:.3f} kg at {f(c)}')
    print(f'  custom_structure_link: {link[0]:.3f} kg at {f(link[1])}; {i(link[2])}')
    print(f"rest settings: front {d['suspension_rest']['front']}, rear {d['suspension_rest']['rear']}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    g = ap.add_mutually_exclusive_group()
    g.add_argument('--check', action='store_true')
    g.add_argument('--report', action='store_true')
    a = ap.parse_args(argv)
    d = load()
    text = generate(d)
    if a.report:
        report(d)
    elif a.check:
        current = OUT.read_text(encoding='utf-8') if OUT.exists() else ''
        if current != text:
            print(f'{OUT} is stale: run {Path(__file__).name}', file=sys.stderr)
            return 1
        print(f'{OUT.name} is current')
    else:
        OUT.write_text(text, encoding='utf-8')
        print(f'wrote {OUT}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
