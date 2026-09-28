# Outdoor terrain: sources

Built by `scripts/make_outdoor_terrain.py` (Blender, seed 7) from assets by
[Poly Haven](https://polyhaven.com), all released under
[CC0](https://creativecommons.org/publicdomain/zero/1.0/): no attribution is
required, and this note is a courtesy.

| asset | used for |
|---|---|
| [dry_ground_rocks](https://polyhaven.com/a/dry_ground_rocks) | surface displacement and ground texture |
| [rock_07](https://polyhaven.com/a/rock_07), [rock_09](https://polyhaven.com/a/rock_09), [stone_01](https://polyhaven.com/a/stone_01) | stones |
| [moon_rock_01](https://polyhaven.com/a/moon_rock_01), [03](https://polyhaven.com/a/moon_rock_03), [04](https://polyhaven.com/a/moon_rock_04), [05](https://polyhaven.com/a/moon_rock_05), [06](https://polyhaven.com/a/moon_rock_06), [07](https://polyhaven.com/a/moon_rock_07) | stones (shape only; rescaled) |

The stones are decimated to about 180 triangles and keep only their colour
maps, downsized to 512 px. `terrain.json` records the generator's
parameters, the stone positions and the resulting height and slope
statistics.
