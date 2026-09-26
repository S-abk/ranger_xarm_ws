#!/usr/bin/env python3
"""Split the Ranger CAD mesh into a body, four wheels and four knuckles.

ranger_mini_v3_cad.stl is the whole vehicle as one mesh, wheels included,
and it hangs off base_link. So the wheels you see are welded to the
chassis and cannot turn, while the wheel LINKS that actually rotate and
steer carry no <visual> at all: they are a collision sphere and an
inertial. The robot drives correctly and looks completely static.

Giving the wheel links a plain cylinder would not fix it either. A
featureless cylinder rotating about its own axis is indistinguishable
from a stationary one; it is the spokes and tread that make rotation
visible. So the wheels are cut out of the CAD itself and handed to the
links they belong to, which keeps the original geometry rather than
substituting a primitive for it.

The wheels are separate connected components in the STL, as CAD assembly
exports usually are, so the split is exact: no triangle is invented,
moved between parts by a distance threshold, or lost. Each wheel is
re-origined on its own axis of rotation, fitted from the tyre itself
rather than assumed, so it spins true instead of wobbling.

The steering knuckles get the same treatment, for the same reason: with
only the wheel split out, the wheel turned but the casting above it did
not, so the corner sheared visibly at full lock.

Which parts belong to a knuckle is less obvious than which belong to a
wheel, because a corner is thirty-odd components (flange, bolt circle,
motor housing, upright) rather than one. They are selected by distance
from the steering axis, and the threshold is not arbitrary: between 20
and 40 mm the triangle count per corner sits on a plateau at ~13050,
varying by 0.2% between corners, which is tessellation noise on a
symmetric platform. Widening to 60 mm jumps the count and makes the four
corners genuinely different (14547 vs 15739), which is chassis being
dragged in. 30 mm sits in the middle of the plateau.

Run from the package root; writes into meshes/visual/.
"""
import argparse
import os
import struct
import sys

import numpy as np

# Wheel stations in the description (metres), from ranger_wheels.xacro.
HALF_WHEELBASE = 0.2470
HALF_TRACK = 0.1852
GROUND_Z = -0.327028595
WHEEL_RADIUS = 0.100036
AXLE_Z = GROUND_Z + WHEEL_RADIUS
MM = 1000.0

CORNERS = {
    'front_left': (+HALF_WHEELBASE, +HALF_TRACK),
    'front_right': (+HALF_WHEELBASE, -HALF_TRACK),
    'rear_left': (-HALF_WHEELBASE, +HALF_TRACK),
    'rear_right': (-HALF_WHEELBASE, -HALF_TRACK),
}


def read_stl(path):
    with open(path, 'rb') as f:
        f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        raw = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50).copy()
    normals = raw[:, 0:12].copy().view('<f4').reshape(n, 3)
    tris = raw[:, 12:48].copy().view('<f4').reshape(n, 3, 3)
    attrs = raw[:, 48:50].copy()
    return normals, tris, attrs


def write_stl(path, normals, tris, attrs, header=b'ranger_xarm_description split'):
    n = len(tris)
    out = np.zeros((n, 50), dtype=np.uint8)
    out[:, 0:12] = normals.astype('<f4').view(np.uint8).reshape(n, 12)
    out[:, 12:48] = tris.astype('<f4').view(np.uint8).reshape(n, 36)
    out[:, 48:50] = attrs
    with open(path, 'wb') as f:
        f.write(header.ljust(80, b'\0')[:80])
        f.write(struct.pack('<I', n))
        f.write(out.tobytes())


def connected_components(tris):
    """Label triangles by welded-vertex connectivity."""
    n = len(tris)
    q = np.round(tris.reshape(-1, 3).astype(np.float64), 3)
    _, inv = np.unique(q, axis=0, return_inverse=True)
    inv = inv.reshape(n, 3)
    nv = int(inv.max()) + 1

    parent = np.arange(nv)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for a, b, c in inv:
        ra, rb, rc = find(a), find(b), find(c)
        if ra != rb:
            parent[rb] = ra
        if ra != rc:
            parent[rc] = ra
    roots = np.array([find(i) for i in range(nv)])
    return roots[inv[:, 0]]


def fit_axis_centre(pts):
    """Least-squares circle centre in the x-z plane (Kasa fit).

    The tyre is a surface of revolution about Y, so its outer profile
    gives the true axis. Taking the bounding-box centre instead is off by
    a couple of millimetres here, which is a visible wobble once the
    wheel turns.
    """
    x, z = pts[:, 0], pts[:, 2]
    A = np.column_stack([x, z, np.ones(len(x))])
    b = x ** 2 + z ** 2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    return sol[0] / 2.0, sol[1] / 2.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mesh', default='meshes/visual/ranger_mini_v3_cad.stl')
    ap.add_argument('--outdir', default='meshes/visual')
    ap.add_argument('--knuckle-radius', type=float, default=30.0,
                    help='mm from the steering axis; components whose centroid '
                         'falls inside belong to the knuckle (plateau is 20-40)')
    args = ap.parse_args()

    if not os.path.exists(args.mesh):
        sys.exit(f'{args.mesh} not found; run this from the package root')

    normals, tris, attrs = read_stl(args.mesh)
    print(f'{args.mesh}: {len(tris)} triangles')

    label = connected_components(tris)
    centroids = tris.mean(axis=1)

    # Match each wheel station to the component whose triangles cluster
    # around it. Identify by component, not by a radius cutoff: the
    # suspension sits inside the same envelope and a cutoff would slice
    # through it.
    used = []
    for name, (lx, ly) in CORNERS.items():
        wx, wy, wz = lx * MM, ly * MM, AXLE_Z * MM
        d = np.linalg.norm(centroids - np.array([wx, wy, wz]), axis=1)
        near = label[d < 120.0]
        if len(near) == 0:
            sys.exit(f'no geometry near the {name} wheel station')
        comp = np.bincount(near.astype(np.int64)).argmax()
        mask = label == comp
        pts = tris[mask].reshape(-1, 3).astype(np.float64)

        # Fit the axis on the outer profile only (the tyre), not the hub.
        r = np.hypot(pts[:, 0] - wx, pts[:, 2] - wz)
        outer = pts[r > 0.85 * r.max()]
        cx, cz = fit_axis_centre(outer)

        # Re-origin on the wheel LINK frame: the fitted axis in x/z, the
        # nominal station in y, so the mesh lands where the link is.
        shift = np.array([cx, wy, cz])
        out = tris[mask] - shift
        path = os.path.join(args.outdir, f'ranger_wheel_{name}.stl')
        write_stl(path, normals[mask], out, attrs[mask])
        print(f'  {name:12s} {mask.sum():6d} tris  axis fitted at '
              f'x={cx:8.3f} z={cz:8.3f} mm  (station x={wx:.1f} z={wz:.1f})'
              f' -> {os.path.basename(path)}')
        used.append(mask)

        # The knuckle: everything else clustered on this steering axis.
        # Shifted by the same vector as the wheel, so the two stay aligned
        # with each other and with the link they hang off.
        kmask = np.zeros(len(tris), dtype=bool)
        for comp in np.unique(label):
            cm = label == comp
            if cm is None or (cm & mask).any():
                continue
            ctr = centroids[cm].mean(axis=0)
            if np.hypot(ctr[0] - wx, ctr[1] - wy) >= args.knuckle_radius:
                continue
            if tris[cm].reshape(-1, 3)[:, 2].min() < -320.0:
                continue     # reaches the ground: that is a wheel, not a knuckle
            kmask |= cm
        if kmask.any():
            kpath = os.path.join(args.outdir, f'ranger_knuckle_{name}.stl')
            write_stl(kpath, normals[kmask], tris[kmask] - shift, attrs[kmask])
            print(f'  {name:12s} {kmask.sum():6d} tris  knuckle (r<'
                  f'{args.knuckle_radius:.0f} mm of the steer axis)'
                  f' -> {os.path.basename(kpath)}')
            used.append(kmask)

    body = ~np.any(used, axis=0)
    path = os.path.join(args.outdir, 'ranger_mini_v3_cad_body.stl')
    write_stl(path, normals[body], tris[body], attrs[body])
    print(f'  body         {body.sum():6d} tris -> {os.path.basename(path)}')
    assert body.sum() + sum(m.sum() for m in used) == len(tris), 'triangles lost'
    print('all triangles accounted for')


if __name__ == '__main__':
    main()
