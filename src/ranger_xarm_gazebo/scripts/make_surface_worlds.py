#!/usr/bin/env python3
"""Generate ground-surface variants of empty_ground.sdf.

Dead reckoning on this platform tracks ground truth to under a tenth of a
percent, which is not a compliment to the odometry: it means the
simulated floor does not let the wheels slip. Real odometry drifts
because real wheels lose grip, and an estimator that is only ever shown a
perfect floor has not been tested against the thing it exists to survive.

These worlds break that, three different ways, because they fail
differently:

  low_friction  uniform low grip. Wheels spin under acceleration and
                lock under braking, so error enters along the direction
                of travel and scales with how hard the base is driven.

  rough         grip is fine but contact is not. Low bumps make wheels
                unload and briefly leave the floor, so a wheel that is
                turning is not always a wheel that is travelling. Error
                arrives in bursts rather than accumulating smoothly.

  mixed         grip varies across the floor. Wheels on different patches
                slip by different amounts at the same instant, which is
                the case that produces HEADING error rather than distance
                error, and heading error is the one that never washes out.

torsional friction stays at zero in all of them, for the reason given in
empty_ground.sdf: gz's patch-radius model grossly overstates it for a
100 kg platform and the base stops being able to yaw at all.

Positions are fixed rather than random, so a run is comparable with the
run before it.
"""
import argparse
import math
import os
import re


def set_friction(sdf, mu):
    """Rewrite the ground plane's mu/mu2."""
    return re.sub(r'(<ode>\s*<mu>)[\d.]+(</mu>\s*<mu2>)[\d.]+(</mu2>)',
                  rf'\g<1>{mu}\g<2>{mu}\g<3>', sdf, count=1)


def bump(name, x, y, z, sx, sy, sz, mu=1.5):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <surface>
            <friction>
              <ode><mu>{mu}</mu><mu2>{mu}</mu2></ode>
              <torsional><coefficient>0.0</coefficient></torsional>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>{0.25 + 0.4*(mu<1):.2f} 0.3 0.35 1</ambient>
            <diffuse>{0.3 + 0.4*(mu<1):.2f} 0.35 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


# The rounded twin of the rough-ground bump: same 24 mm height, same
# 0.16 m footprint, no vertical face. A sphere sunk into the floor so only
# its cap shows; the radius and depth follow from the cap geometry,
# R = (a^2 + h^2) / 2h with a the footprint half-width and h the height.
# It exists to test one thing -- whether the gz/Isaac rough-ground
# disagreement comes from a tyre sphere striking a box EDGE -- so every
# other property is held equal to bump().
BUMP_H = 0.024
BUMP_HALF = 0.08
DOME_R = (BUMP_HALF ** 2 + BUMP_H ** 2) / (2 * BUMP_H)
DOME_Z = BUMP_H - DOME_R


def dome(name, x, y, mu=1.5):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {DOME_Z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><sphere><radius>{DOME_R:.4f}</radius></sphere></geometry>
          <surface>
            <friction>
              <ode><mu>{mu}</mu><mu2>{mu}</mu2></ode>
              <torsional><coefficient>0.0</coefficient></torsional>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><sphere><radius>{DOME_R:.4f}</radius></sphere></geometry>
          <material>
            <ambient>0.25 0.3 0.35 1</ambient>
            <diffuse>0.30 0.35 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def rough_grid():
    """Bump centres for both rough worlds, so the two cannot drift apart."""
    for ix in range(-3, 9):
        for iy in range(-4, 5):
            # stagger so the four wheels never hit bumps simultaneously
            x = ix * 0.55 + (0.27 if iy % 2 else 0.0)
            y = iy * 0.55
            if abs(x) < 0.7 and abs(y) < 0.7:
                continue          # leave the spawn point flat
            yield x, y


def static_box(name, x, y, z, sx, sy, sz):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def static_cylinder(name, x, y, radius, height):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {height / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>
          <material><ambient>0.2 0.35 0.5 1</ambient><diffuse>0.25 0.4 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


# A room for lidar odometry to see. Every other world is a bare ground
# plane, and scan matching against a plane cannot observe horizontal
# motion: on empty_ground KISS-ICP missed an entire 137 deg turn. The walls
# and the row of round pillars stand well clear of the scored drive,
# whose path including the robot's footprint stays inside x -0.7..3.4,
# y -0.7..2.6, and clear of the rough-ground bump field.
ROOM_X = (-4.0, 7.0)
ROOM_Y = (-4.0, 6.0)
WALL_T, WALL_H = 0.2, 2.0
PILLAR_R, PILLAR_H = 0.6, 1.0
PILLAR_XS = (-2.0, 0.0, 2.0, 4.0)
PILLAR_Y = 4.6


def room_blocks():
    (x0, x1), (y0, y1) = ROOM_X, ROOM_Y
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    lx, ly = x1 - x0 + WALL_T, y1 - y0 + WALL_T
    blocks = [
        static_box('wall_south', cx, y0, WALL_H / 2, lx, WALL_T, WALL_H),
        static_box('wall_north', cx, y1, WALL_H / 2, lx, WALL_T, WALL_H),
        static_box('wall_west', x0, cy, WALL_H / 2, WALL_T, ly, WALL_H),
        static_box('wall_east', x1, cy, WALL_H / 2, WALL_T, ly, WALL_H),
    ]
    blocks += [static_cylinder(f'pillar_{i + 1}', x, PILLAR_Y, PILLAR_R, PILLAR_H)
               for i, x in enumerate(PILLAR_XS)]
    return blocks


def insert(sdf, blocks, header):
    i = sdf.rstrip().rfind('</world>')
    sdf = sdf.replace('<sdf', header + '<sdf', 1)
    i = sdf.rstrip().rfind('</world>')
    return sdf[:i] + ''.join(blocks) + '\n  ' + sdf[i:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', default='worlds/empty_ground.sdf')
    ap.add_argument('--outdir', default='worlds')
    args = ap.parse_args()
    base = open(args.base).read()

    # low friction
    hdr = ("<!-- Uniform low grip (mu 0.25). Generated by "
           "scripts/make_surface_worlds.py; edit that, not this. -->\n")
    open(os.path.join(args.outdir, 'low_friction.sdf'), 'w').write(
        insert(set_friction(base, 0.25), [], hdr))

    # rough: a field of low bumps the wheels ride over
    blocks = []
    n = 0
    for x, y in rough_grid():
        n += 1
        blocks.append(bump(f'bump_{n}', x, y, BUMP_H / 2,
                           2 * BUMP_HALF, 2 * BUMP_HALF, BUMP_H))
    n_rough = n
    hdr = (f"<!-- {n} low bumps, 24 mm tall, staggered so the four wheels do "
           "not strike them together. Grip is normal; it is CONTACT that is "
           "intermittent. Generated by scripts/make_surface_worlds.py. -->\n")
    open(os.path.join(args.outdir, 'rough_ground.sdf'), 'w').write(
        insert(base, blocks, hdr))

    # the same flat floor and rough field, inside a room lidar can see
    room_hdr = ("<!-- {what}, inside an {w:.0f} x {h:.0f} m walled room with a row of "
                "{n} round pillars, so lidar odometry has structure to "
                "match against. Generated by scripts/make_surface_worlds.py. -->\n")
    dims = dict(w=ROOM_X[1] - ROOM_X[0], h=ROOM_Y[1] - ROOM_Y[0], n=len(PILLAR_XS))
    open(os.path.join(args.outdir, 'empty_room.sdf'), 'w').write(
        insert(base, room_blocks(), room_hdr.format(what='Flat floor', **dims)))
    rough_bumps = [bump(f'bump_{i + 1}', x, y, BUMP_H / 2, 2 * BUMP_HALF, 2 * BUMP_HALF, BUMP_H)
                   for i, (x, y) in enumerate(rough_grid())]
    open(os.path.join(args.outdir, 'rough_room.sdf'), 'w').write(
        insert(base, rough_bumps + room_blocks(),
               room_hdr.format(what='The rough_ground.sdf bump field', **dims)))

    # rough, rounded: the same field with domes instead of boxes
    blocks = [dome(f'dome_{i + 1}', x, y) for i, (x, y) in enumerate(rough_grid())]
    hdr = (f"<!-- {len(blocks)} low domes, 24 mm tall, 0.16 m footprint, on the "
           "same staggered grid as rough_ground.sdf. Identical in every respect "
           "except shape: no vertical face for a tyre to strike. Generated by "
           "scripts/make_surface_worlds.py. -->\n")
    open(os.path.join(args.outdir, 'rough_rounded.sdf'), 'w').write(
        insert(base, blocks, hdr))

    # mixed: alternating grip patches, thin enough to drive onto
    blocks = []
    n = 0
    for ix in range(-2, 7):
        for iy in range(-3, 4):
            if (ix + iy) % 2:
                continue
            n += 1
            blocks.append(bump(f'patch_{n}', ix * 1.2, iy * 1.2, 0.004,
                               1.1, 1.1, 0.008, mu=0.15))
    hdr = (f"<!-- {n} low-grip patches (mu 0.15) on a normal floor, in a "
           "checker so wheels on the same axle can be on different surfaces. "
           "This is the case that produces heading error. Generated by "
           "scripts/make_surface_worlds.py. -->\n")
    open(os.path.join(args.outdir, 'mixed_surface.sdf'), 'w').write(
        insert(base, blocks, hdr))

    print(f'low_friction.sdf  mu 0.25')
    print(f'rough_ground.sdf / rough_rounded.sdf  {n_rough} bumps each')
    print(f'mixed_surface.sdf {n} patches at mu 0.15')


if __name__ == '__main__':
    main()
