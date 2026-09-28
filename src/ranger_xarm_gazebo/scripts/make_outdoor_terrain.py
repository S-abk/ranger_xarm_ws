#!/usr/bin/env python3
"""Build an outdoor rough-terrain mesh in Blender, for both simulators.

    blender -b -P make_outdoor_terrain.py -- \
        --out ../models/outdoor_terrain [--cache ~/.cache/ranger_xarm_terrain] [--seed 7]

The bump fields in make_surface_worlds.py sit on a lattice, and the robot's
wheels (y = +/-0.185 m) run straight between its rows (y = 0, +/-0.55 m,
each +/-0.08 m wide): on the scored drive a wheel is on a bump for only a
quarter of the distance, and never on the first straight. Real ground has
no lattice. This builds one from three layers, none of them periodic at the
scale of the robot:

- undulation: band-limited noise, wavelengths 1.5 - 10 m, 3 cm standard
  deviation, so the chassis pitches and rolls as it drives;
- surface: the displacement map of Poly Haven's scanned `dry_ground_rocks`
  (a 4 x 4 m patch of real ground), averaged to the mesh's 4 cm spacing,
  plus fine noise so the 4 m tile does not repeat visibly;
- stones: Poly Haven's scanned rocks, rescaled, rotated at random, partly
  buried and scattered by Poisson-disc sampling, standing 1.5 - 5 cm proud.
  5 cm is half the 10 cm wheel radius, the most a wheel climbs cleanly.

A flat pad at the origin keeps resets repeatable, and the edge fades to the
ground plane. Heights are offset so nothing dips below z = 0, where the
ground plane would show through.

The same triangles go to terrain.glb (gz: visual and collision) and
terrain.usdc (Isaac), so the two engines drive over identical geometry.
All assets are CC0 (polyhaven.com); see ATTRIBUTION.md next to the output.
The outputs are committed, so Blender is only needed to change the terrain.
"""
import argparse
import json
import math
import os
import sys
import urllib.request

import bpy
import numpy as np

# The room of make_surface_worlds.py, so the terrain fills it exactly.
X0, X1, Y0, Y1 = -4.0, 7.0, -4.0, 6.0
RES = 0.04                      # m between terrain vertices
UNDULATION_STD = 0.03           # m
SURFACE_STD = 0.006             # m, from the scanned displacement
FINE_STD = 0.003                # m
PAD_R, PAD_BLEND = 0.45, 0.35   # m, flat spawn pad
EDGE_FADE = 0.8                 # m
STONE_DENSITY = 2.5             # per m^2, outside the pad
STONE_MIN_GAP = 0.22            # m between stone centres
STONE_PROUD = (0.015, 0.05)     # m above the local surface
STONE_SIZE = (0.07, 0.22)       # m, longest horizontal dimension
STONE_TRIS = 180                # per stone after decimation
STONE_TEXTURE_PX = 512          # colour map only; normal/ORM maps dropped
TERRAIN_DECIMATE = 0.25
GROUND_TEXTURE = 'dry_ground_rocks'
GROUND_TILE = 4.0               # m, the texture's real-world size
STONES = ['rock_07', 'rock_09', 'stone_01', 'moon_rock_01', 'moon_rock_03',
          'moon_rock_04', 'moon_rock_05', 'moon_rock_06', 'moon_rock_07']
API = 'https://api.polyhaven.com'
# Poly Haven's API refuses urllib's default User-Agent and asks clients to
# identify themselves.
HEADERS = {'User-Agent': 'ranger_xarm_ws-terrain-generator/1.0'}


def get(url, timeout=60):
    return urllib.request.urlopen(urllib.request.Request(url, headers=HEADERS), timeout=timeout)


def args_after_dashes():
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--cache', default=os.path.expanduser('~/.cache/ranger_xarm_terrain'))
    ap.add_argument('--seed', type=int, default=7)
    return ap.parse_args(argv)


def fetch(url, path):
    if not os.path.exists(path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with get(url) as r, open(path + '.part', 'wb') as f:
            f.write(r.read())
        os.replace(path + '.part', path)
    return path


def fetch_model(name, cache):
    """A Poly Haven model's 1k glTF with its buffers and textures."""
    files = json.load(get(f'{API}/files/{name}'))
    g = files['gltf']['1k']['gltf']
    root = os.path.join(cache, 'models', name)
    main = fetch(g['url'], os.path.join(root, os.path.basename(g['url'])))
    for rel, inc in g['include'].items():
        fetch(inc['url'], os.path.join(root, rel))
    return main


def fetch_texture(name, cache):
    files = json.load(get(f'{API}/files/{name}'))
    out = {}
    for key, res, fmt in (('Diffuse', '1k', 'jpg'), ('Displacement', '2k', 'png')):
        url = files[key][res][fmt]['url']
        out[key] = fetch(url, os.path.join(cache, 'textures', os.path.basename(url)))
    return out


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3 - 2 * t)


def band_noise(rng, ny, nx, lam_min, lam_max, beta):
    """Unit-variance noise with a k^-beta spectrum between two wavelengths."""
    f = np.fft.rfft2(rng.standard_normal((ny, nx)))
    k = np.hypot(np.fft.rfftfreq(nx, RES)[None, :], np.fft.fftfreq(ny, RES)[:, None])
    band = (k >= 1.0 / lam_max) & (k <= 1.0 / lam_min)
    h = np.fft.irfft2(f * np.where(band, np.maximum(k, 1e-9) ** (-beta / 2), 0.0), s=(ny, nx))
    return h / h.std()


def scanned_surface(path, xs, ys, rng):
    """The scanned displacement, block-averaged to RES and tiled every GROUND_TILE."""
    img = bpy.data.images.load(path)
    w, h = img.size
    px = np.array(img.pixels[:], dtype=np.float32).reshape(h, w, img.channels)[:, :, 0]
    bpy.data.images.remove(img)
    n = int(round(GROUND_TILE / RES))           # cells per tile
    b = w // n                                  # pixels per cell
    tile = px[:n * b, :n * b].reshape(n, b, n, b).mean(axis=(1, 3))
    tile = (tile - tile.mean()) / tile.std()
    ox, oy = rng.integers(0, n, 2)              # where the tile starts
    ix = (np.round((xs - X0) / RES).astype(int) + ox) % n
    iy = (np.round((ys - Y0) / RES).astype(int) + oy) % n
    return tile[np.ix_(iy, ix)]


def heightfield(rng, ground_disp):
    xs = np.arange(X0, X1 + RES / 2, RES)
    ys = np.arange(Y0, Y1 + RES / 2, RES)
    ny, nx = len(ys), len(xs)
    h = (UNDULATION_STD * band_noise(rng, ny, nx, 1.5, 10.0, 3.0)
         + SURFACE_STD * scanned_surface(ground_disp, xs, ys, rng)
         + FINE_STD * band_noise(rng, ny, nx, 0.12, 1.0, 2.0))
    X, Y = np.meshgrid(xs, ys)
    pad = smoothstep(PAD_R, PAD_R + PAD_BLEND, np.hypot(X, Y))
    h = h * pad + h[np.hypot(X, Y) < PAD_R].mean() * (1 - pad)
    edge = np.minimum.reduce([X - X0, X1 - X, Y - Y0, Y1 - Y])
    fade = smoothstep(0.0, EDGE_FADE, edge)
    offset = 0.005 - min(0.0, h.min())
    h = (h + offset) * fade
    return xs, ys, h


def sample(xs, ys, h, x, y):
    i = int(round((x - X0) / RES))
    j = int(round((y - Y0) / RES))
    return float(h[min(max(j, 0), len(ys) - 1), min(max(i, 0), len(xs) - 1)])


def terrain_object(xs, ys, h, diffuse):
    ny, nx = h.shape
    X, Y = np.meshgrid(xs, ys)
    verts = np.stack([X.ravel(), Y.ravel(), h.ravel()], axis=1)
    idx = np.arange(nx * ny).reshape(ny, nx)
    a, b, c, d = idx[:-1, :-1], idx[:-1, 1:], idx[1:, 1:], idx[1:, :-1]
    quads = np.stack([a, b, c, d], axis=-1).reshape(-1, 4)
    mesh = bpy.data.meshes.new('terrain')
    mesh.from_pydata(verts.tolist(), [], quads.tolist())
    uv = mesh.uv_layers.new(name='UVMap')
    loop_v = np.empty(len(mesh.loops), dtype=np.int64)
    mesh.loops.foreach_get('vertex_index', loop_v)
    uvs = np.stack([(verts[loop_v, 0] - X0) / GROUND_TILE,
                    (verts[loop_v, 1] - Y0) / GROUND_TILE], axis=1)
    uv.data.foreach_set('uv', uvs.ravel())
    obj = bpy.data.objects.new('terrain', mesh)
    bpy.context.collection.objects.link(obj)

    mat = bpy.data.materials.new('ground')
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes['Principled BSDF']
    bsdf.inputs['Roughness'].default_value = 0.95
    tex = mat.node_tree.nodes.new('ShaderNodeTexImage')
    tex.image = bpy.data.images.load(diffuse)
    mat.node_tree.links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])
    mesh.materials.append(mat)

    dec = obj.modifiers.new('decimate', 'DECIMATE')
    dec.ratio = TERRAIN_DECIMATE
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_apply(modifier='decimate')
    return obj


def slim_material(obj, cache):
    """Keep only the colour map, resaved small: a stone is a few cm across,
    and three 1k maps for each of nine stones made a 21 MB mesh."""
    for mat in obj.data.materials:
        nodes, links = mat.node_tree.nodes, mat.node_tree.links
        bsdf = next(n for n in nodes if n.type == 'BSDF_PRINCIPLED')
        # Poly Haven's colour reaches Base Color through a Mix with the
        # vertex colours; walk upstream to the first image.
        img, todo = None, [bsdf.inputs['Base Color']]
        while todo and img is None:
            sock = todo.pop()
            for link in sock.links:
                if link.from_node.type == 'TEX_IMAGE':
                    img = link.from_node.image
                    break
                todo.extend(link.from_node.inputs)
        for n in [n for n in nodes if n.type not in ('BSDF_PRINCIPLED', 'OUTPUT_MATERIAL')]:
            nodes.remove(n)
        if img is None:
            continue
        img.pixels[0]           # scale() is a no-op on pixels not yet loaded
        img.scale(STONE_TEXTURE_PX, STONE_TEXTURE_PX)
        os.makedirs(cache, exist_ok=True)
        small = os.path.join(cache, f'{mat.name}_diff_{STONE_TEXTURE_PX}.jpg')
        img.filepath_raw = small
        img.file_format = 'JPEG'
        img.save()
        # Both exporters copy an image's original file when it has one, so
        # hand them the small file as an image of its own.
        tex = nodes.new('ShaderNodeTexImage')
        tex.image = bpy.data.images.load(small)
        links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])
        bsdf.inputs['Roughness'].default_value = 0.9


def stone_prototypes(cache):
    protos = []
    for name in STONES:
        before = set(bpy.data.objects)
        bpy.ops.import_scene.gltf(filepath=fetch_model(name, cache))
        new = [o for o in set(bpy.data.objects) - before if o.type == 'MESH']
        for o in set(bpy.data.objects) - before:
            if o.type != 'MESH':
                bpy.data.objects.remove(o)
        obj = new[0]
        obj.parent = None
        slim_material(obj, os.path.join(cache, 'slim'))
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        tris = sum(len(p.vertices) - 2 for p in obj.data.polygons)
        if tris > STONE_TRIS:
            dec = obj.modifiers.new('decimate', 'DECIMATE')
            dec.ratio = STONE_TRIS / tris
            bpy.ops.object.modifier_apply(modifier='decimate')
        # Centre on the bounding box, bottom unchanged relative to the centre.
        v = np.array([vv.co[:] for vv in obj.data.vertices])
        centre = (v.max(axis=0) + v.min(axis=0)) / 2
        for vv in obj.data.vertices:
            vv.co = tuple(np.array(vv.co[:]) - centre)
        obj.hide_set(True)
        protos.append(obj)
    return protos


def poisson(rng, n_target):
    pts, tries = [], 0
    while len(pts) < n_target and tries < n_target * 60:
        tries += 1
        x = rng.uniform(X0 + EDGE_FADE, X1 - EDGE_FADE)
        y = rng.uniform(Y0 + EDGE_FADE, Y1 - EDGE_FADE)
        if math.hypot(x, y) < PAD_R + PAD_BLEND + 0.15:
            continue
        if all((x - px) ** 2 + (y - py) ** 2 >= STONE_MIN_GAP ** 2 for px, py in pts):
            pts.append((x, y))
    return pts


def scatter_stones(rng, protos, xs, ys, h):
    area = (X1 - X0 - 2 * EDGE_FADE) * (Y1 - Y0 - 2 * EDGE_FADE)
    placed = []
    for i, (x, y) in enumerate(poisson(rng, int(STONE_DENSITY * area))):
        proto = protos[rng.integers(len(protos))]
        obj = proto.copy()      # shares the mesh: glTF writes it once
        obj.name = f'stone_{i:03d}'
        bpy.context.collection.objects.link(obj)
        obj.hide_set(False)
        dims = np.array(proto.dimensions[:])
        s = rng.uniform(*STONE_SIZE) / max(dims[0], dims[1])
        proud = rng.uniform(*STONE_PROUD)
        # Tall enough that some of it is always buried.
        s = max(s, 1.2 * proud / dims[2])
        obj.scale = (s, s, s)
        obj.rotation_euler = (rng.normal(0, 0.15), rng.normal(0, 0.15), rng.uniform(0, 2 * math.pi))
        bpy.context.view_layer.update()
        top = max((obj.matrix_world @ v.co).z for v in obj.data.vertices)
        ground = sample(xs, ys, h, x, y)
        obj.location = (x, y, ground + proud - top)
        placed.append({'x': round(x, 3), 'y': round(y, 3), 'proud': round(proud, 4)})
    return placed


def main():
    a = args_after_dashes()
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    rng = np.random.default_rng(a.seed)
    bpy.ops.wm.read_factory_settings(use_empty=True)

    ground = fetch_texture(GROUND_TEXTURE, a.cache)
    xs, ys, h = heightfield(rng, ground['Displacement'])
    terrain = terrain_object(xs, ys, h, ground['Diffuse'])
    protos = stone_prototypes(a.cache)
    stones = scatter_stones(rng, protos, xs, ys, h)
    for p in protos:
        bpy.data.objects.remove(p)

    bpy.ops.object.select_all(action='SELECT')
    tris = sum(sum(len(p.vertices) - 2 for p in o.data.polygons)
               for o in bpy.data.objects if o.type == 'MESH')
    # Z up in the file: gz and Isaac are both Z-up, and glTF's default
    # Y-up conversion would have to be undone on load.
    bpy.ops.export_scene.gltf(filepath=os.path.join(out, 'terrain.glb'),
                              export_format='GLB', export_yup=False,
                              export_apply=True, use_selection=False,
                              export_image_format='JPEG')
    bpy.ops.wm.usd_export(filepath=os.path.join(out, 'terrain.usdc'),
                          export_global_up_selection='Z', root_prim_path='/terrain',
                          export_materials=True, generate_preview_surface=True,
                          export_textures_mode='NEW', overwrite_textures=True,
                          relative_paths=True, triangulate_meshes=True)

    def slope(baseline):
        # Over a baseline, not per 4 cm cell: a 10 cm wheel rides over
        # pebble-scale roughness rather than following it.
        k = max(1, int(round(baseline / RES)))
        gy, gx = np.gradient(h[::k, ::k], k * RES)
        sl = np.degrees(np.arctan(np.hypot(gx, gy)))
        return {'p50': round(float(np.percentile(sl, 50)), 2),
                'p99': round(float(np.percentile(sl, 99)), 2)}
    meta = {
        'seed': a.seed, 'extent_m': [X0, X1, Y0, Y1], 'resolution_m': RES,
        'height_m': {'min': float(h.min()), 'max': float(h.max()),
                     'pad': sample(xs, ys, h, 0.0, 0.0)},
        'slope_deg_over_0.2m': slope(0.2),
        'slope_deg_over_0.5m': slope(0.5),
        'stones': len(stones), 'triangles': int(tris),
        'assets': {'ground': GROUND_TEXTURE, 'stones': STONES},
    }
    with open(os.path.join(out, 'terrain.json'), 'w') as f:
        json.dump(dict(meta, stone_positions=stones), f, indent=1)
    print('TERRAIN', json.dumps(meta), flush=True)


main()
