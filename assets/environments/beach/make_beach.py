#!/usr/bin/env python3
"""A sand beach on the sea: an environment for robots and vehicles.

Writes, next to this script:

* beach.tiff: elevation of the beach, 120 m along the shore (x) by 100 m
  across it (y), on a 0.5 m grid: dunes behind (-y), a dry upper beach, a
  wet foreshore sloping into the sea (+y), and the sea floor. 32 bit float
  samples, z (down) of the surface in the terrain frame, columns along x
  ("coordinate_type": "sae").
* beach_scenery.tiff: the same, a little lower under the deformable strip
  (see below), so that the strip's surface, with its footprints, is drawn
  over it.
* beach_materials.tiff: the surface material of each 0.25 m square, 8 bit:
  0 dry sand, 1 wet sand, 2 dune grass, 3 the wet sand of the strip (as the
  wet sand, but kept clear of rocks), 4 damp sand between the wet and the
  dry.

The beach is two terrains on one frame: the basic terrain "beach" for the
scenery, and the deformable SCM terrain "strip", 13 m along the shore on the
wet sand, where feet and wheels leave their prints. Boulders, rocks and
stones lie about the beach (../../scenery), clear of the strip. The sea is
the environment's water, at height 0 (the inertial frame's z = 0); the sky
is the physical sky.

A model stands on the beach with the root keys of beach_parts(), its
frame, terrains, surface materials, scenery and environment: import this
script, as examples/beach/make_strandbeest_beach.py does.
"""

import os

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
SCENERY = os.path.join(HERE, "..", "..", "scenery")

SPACING = 0.5                      # m, of the elevation map
X0, Y0 = -60.0, -40.0              # world x, y of the terrain frame's origin
COLUMNS, ROWS = 241, 201           # 120 m along x, 100 m along y
MATERIAL_SPACING = 0.25            # m, of the material map
STRIP_X = (-3.0, 10.0)             # world extent of the deformable strip
STRIP_Y = (-2.0, 2.0)
STRIP_SPACING = 0.04
STRIP_DIP = 0.08                   # m the scenery is lowered under the strip


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def value_noise(x, y, scale, seed):
    """smooth noise from -1 to 1 with features about scale (m) across, from
    a fixed seed so that the beach is the same every time"""
    rng = np.random.default_rng(seed)
    grid = rng.uniform(-1.0, 1.0, (64, 64))
    u, v = x / scale, y / scale
    i, j = np.floor(u).astype(int), np.floor(v).astype(int)
    fu, fv = u - i, v - j
    fu, fv = fu * fu * (3 - 2 * fu), fv * fv * (3 - 2 * fv)
    g = lambda a, b: grid[a % 64, b % 64]
    return ((g(i, j) * (1 - fu) + g(i + 1, j) * fu) * (1 - fv) +
            (g(i, j + 1) * (1 - fu) + g(i + 1, j + 1) * fu) * fv)


def height(x, y):
    """height of the beach above the sea (m, upward) at world x, y"""
    # the shoreline wanders along the beach, though not by the strip
    calm = smoothstep(14.0, 30.0, np.abs(x - 3.0))
    y = y - calm * (2.5 * np.sin(x / 23.0) + 1.0 * np.sin(x / 9.0 + 1.0))
    # the foreshore: wet sand at 4 % from the strandline (y = -3) down to
    # the waterline (y = 6), and on under the sea, levelling off
    h = 0.04 * (6.0 - y)
    under = y > 6.0
    h = np.where(under, -0.04 * (y - 6.0) - 0.0012 * (y - 6.0) ** 2, h)
    h = np.maximum(h, -6.0)
    # and falling away offshore, deep enough by the edges of the terrain
    # (y = 60, x = -60 and 60) that the floor no longer shows through the
    # water, which would end there in a line
    offshore = np.maximum(smoothstep(12.0, 62.0, y),
                          smoothstep(15.0, 62.0, np.abs(x)) *
                          smoothstep(6.0, 22.0, y))
    h -= 26.0 * offshore
    # the dry upper beach, steeper, up to the foot of the dunes at y = -14
    h = np.where(y < -3.0, 0.36 + 0.06 * (-3.0 - y), h)
    # the dunes, rising behind it in mounds
    dunes = smoothstep(-14.0, -24.0, y)
    mounds = 1.0 + 0.6 * value_noise(x, y, 11.0, 7) + 0.3 * value_noise(
        x, y, 4.0, 8)
    h = h + dunes * 2.6 * mounds
    # small undulations of the sand, none on the strip
    strip = (smoothstep(STRIP_X[0] - 2.0, STRIP_X[0], x) *
             smoothstep(STRIP_X[1] + 2.0, STRIP_X[1], x) *
             smoothstep(STRIP_Y[0] - 2.0, STRIP_Y[0], y) *
             smoothstep(STRIP_Y[1] + 2.0, STRIP_Y[1], y))
    h = h + (1.0 - strip) * 0.04 * value_noise(x, y, 3.0, 9)
    return h


def grid(spacing, columns, rows):
    x = X0 + spacing * np.arange(columns)
    y = Y0 + spacing * np.arange(rows)
    return np.meshgrid(x, y)  # rows along y, columns along x


def write_maps():
    x, y = grid(SPACING, COLUMNS, ROWS)
    h = height(x, y)
    Image.fromarray((-h).astype(np.float32), mode="F").save(
        os.path.join(HERE, "beach.tiff"))
    # the scenery under the strip, lowered inside its edges
    inside = (smoothstep(STRIP_X[0], STRIP_X[0] + 1.0, x) *
              smoothstep(STRIP_X[1], STRIP_X[1] - 1.0, x) *
              smoothstep(STRIP_Y[0], STRIP_Y[0] + 1.0, y) *
              smoothstep(STRIP_Y[1], STRIP_Y[1] - 1.0, y))
    Image.fromarray((-(h - STRIP_DIP * inside)).astype(np.float32),
                    mode="F").save(os.path.join(HERE, "beach_scenery.tiff"))

    columns = int(round((COLUMNS - 1) * SPACING / MATERIAL_SPACING)) + 1
    rows = int(round((ROWS - 1) * SPACING / MATERIAL_SPACING)) + 1
    x, y = grid(MATERIAL_SPACING, columns, rows)
    h = height(x, y)
    material = np.zeros(h.shape, np.uint8)  # dry sand
    # wet below the reach of the waves, with a ragged edge
    edge = 0.38 + 0.05 * value_noise(x, y, 2.0, 11)
    # damp above it, drying out over a band of 10 to 20 cm of height
    damp = h < edge + 0.15 + 0.05 * value_noise(x, y, 1.5, 12)
    material[damp] = 4
    material[h < edge] = 1
    # the strip, wet sand of its own so that the rocks keep clear of it
    strip = ((x >= STRIP_X[0]) & (x <= STRIP_X[1]) & (y >= STRIP_Y[0]) &
             (y <= STRIP_Y[1]))
    material[strip] = 3
    # grass on the dunes
    grass = (h > 1.5) & (value_noise(x, y, 2.5, 14) +
                         0.5 * value_noise(x, y, 0.8, 15) > -0.1)
    material[grass] = 2
    Image.fromarray(material, mode="L").save(
        os.path.join(HERE, "beach_materials.tiff"))


def relative(path, model_dir):
    """path from the directory of the model file, ending in /"""
    return os.path.relpath(path, model_dir).replace(os.sep, "/") + "/"


def beach_parts(model_dir):
    """the root keys of the beach: its frame, terrains, surface materials,
    scenery and environment, their files named from model_dir, the
    directory of the model file"""
    include = relative(HERE, model_dir)
    textures = include + "textures/"
    scenery = relative(SCENERY, model_dir)

    def meshes(names):
        return [{"type": "mesh", "filename": [scenery + n + ".glb"]}
                for n in names]

    # one scan of sand (Poly Haven's Sand 01, CC0), dry, damp and wet: water
    # in the pores darkens sand to about half its dry reflectance, the more
    # in the blue, so the wet sand is darker and more saturated, and its
    # film of water smooths it, so it is glossier and its grains show less
    # (Twomey, Bohren and Mergenthaler, Applied Optics 25, 1986; Lobell and
    # Asner, Soil Sci. Soc. Am. J. 66, 2002)
    dry = [1.12, 1.03, 0.86]

    def sand(darkening, roughness, normal_scale, soil):
        return {"texture_filename": textures + "sand_01_diff.jpg",
                "normal_map_filename": textures + "sand_01_nor_gl.jpg",
                "roughness_map_filename": textures + "sand_01_rough.jpg",
                "height_map_filename": textures + "sand_01_disp.jpg",
                # the ripples of the sand, a centimeter or two deep
                "height_scale": 0.015,
                "texture_scale": 1.5, "normal_scale": normal_scale,
                "color": [round(c * d, 3) for c, d in zip(dry, darkening)],
                "roughness": roughness, "soil": {"name": soil}}

    wet_sand = sand([0.56, 0.52, 0.47], 0.35, 0.45, "lete_sand")
    material_map = {
        "filename": include + "beach_materials.tiff",
        "x_scale": MATERIAL_SPACING, "y_scale": MATERIAL_SPACING,
        "blend_width": 0.3,
        "materials": [{"value": 0, "material": "dry_sand"},
                      {"value": 1, "material": "wet_sand"},
                      {"value": 2, "material": "dune_grass"},
                      {"value": 3, "material": "strip_sand"},
                      {"value": 4, "material": "damp_sand"}]}
    return {
        "frames": [{"name": "beach", "T": [X0, Y0, 0.0]}],
        "environment": {
            "sky": {"type": "physical", "turbidity": 2.5},
            # the afternoon sun over the sea
            "sun": {"azimuth": 125.0, "elevation": 32.0},
            # a low swell rolling in toward the beach (-y), 60 m long, so of
            # a period of 6.2 s (deep water waves, w^2 = g k)
            # and on it the chop of a light breeze blowing onshore
            "water": {"height": 0.0, "wave_height": 0.25, "wavelength": 60.0,
                      "wave_direction": 265.0, "wind_speed": 5.0},
        },
        "surface_materials": [
            {"name": "dry_sand", "label": "sand",
             **sand([1.0, 1.0, 1.0], 0.95, 1.0, "dry_sand")},
            {"name": "damp_sand", "label": "sand",
             **sand([0.76, 0.73, 0.69], 0.7, 0.75, "lete_sand")},
            {"name": "wet_sand", "label": "sand", **wet_sand},
            {"name": "strip_sand", "label": "sand", **wet_sand},
            {"name": "dune_grass", "label": "grass",
             "texture_filename": "qrc:/textures/grass.jpg",
             "normal_map_filename": "qrc:/textures/grass_normal.jpg",
             "texture_scale": 2.0, "color": [0.9, 0.85, 0.6],
             "roughness": 0.9, "soil": {"name": "dry_sand"}},
        ],
        "terrain": [
            {"type": "basic", "name": "beach", "parent": [["fixed"], "beach"],
             "filename": include + "beach_scenery.tiff",
             "x_scale": SPACING, "y_scale": SPACING, "z_scale": 1.0,
             "texture_variation": True, "material_map": material_map},
            {"type": "scm", "name": "strip", "parent": [["fixed"], "beach"],
             "x": [STRIP_X[0] - X0, STRIP_X[1] - X0],
             "y": [STRIP_Y[0] - Y0, STRIP_Y[1] - Y0],
             "spacing": STRIP_SPACING, "settle_time": 1.0,
             "filename": include + "beach.tiff",
             "x_scale": SPACING, "y_scale": SPACING, "z_scale": 1.0,
             "soil": {"name": "lete_sand"},
             "texture_variation": True, "material_map": material_map},
        ],
        # boulders, rocks and stones strewn over the beach and the dunes, in
        # the surf too, clear of the strip (strip_sand) by a margin
        "scenery": [
            {"name": "boulders", "terrain": "beach",
             "materials": ["dry_sand", "damp_sand", "wet_sand",
                           "dune_grass"],
             "margin": 4.0, "density": 0.0015, "spacing": 10.0,
             "scale": [0.25, 0.6], "tilt": 0.5, "depth": 0.3, "seed": 21,
             "draw_distance": 300.0,
             "geometry": meshes(["boulder_03", "boulder_04"])},
            {"name": "rocks", "terrain": "beach",
             "materials": ["dry_sand", "damp_sand", "wet_sand",
                           "dune_grass"],
             "margin": 1.0, "density": 0.04, "spacing": 1.2,
             "avoid": ["boulders"], "scale": [0.8, 2.5], "tilt": 1.0,
             "depth": 0.03, "seed": 22, "draw_distance": 80.0,
             "geometry": meshes(["rock_07", "rock_09", "stone_01"])},
            {"name": "stones", "terrain": "beach",
             "materials": ["dry_sand", "damp_sand", "wet_sand"],
             "margin": 0.6, "density": 0.25, "spacing": 0.4,
             "avoid": ["boulders", "rocks"], "scale": [1.0, 2.2],
             "tilt": 1.0, "depth": 0.01, "seed": 23, "draw_distance": 30.0,
             "geometry": meshes(["stones_a", "stones_b", "stones_c",
                                 "stones_d", "stones_e"])},
        ],
    }


if __name__ == "__main__":
    write_maps()
    print("wrote beach.tiff, beach_scenery.tiff and beach_materials.tiff in",
          HERE)
