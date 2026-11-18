#!/usr/bin/env python3
"""A gravel rally stage through a conifer forest: an environment for
vehicles.

Writes, next to this script:

* forest.tiff: elevation of the stage, 400 m (x) by 300 m (y) on a 0.5 m
  grid: wooded hills, and a gravel road about 850 m long cut into them, with
  a crown, shoulders and ditches, and the bumps of a gravel road; a jump, and
  a crest into an off-camber corner with a run-off (JUMPS, CAMBERS,
  RUN_OFFS). 32 bit float samples, z (down) of the surface in the terrain frame, columns
  along x ("coordinate_type": "sae").
* forest_materials.tiff: the surface material of each 0.25 m square, 8 bit:
  0 forest floor (needles, moss and twigs), 1 loose gravel of the road,
  2 the packed dirt of the wheel lines, 3 the grass of the verges, 4 bare
  rock on the steep banks of the cuttings.

Firs, pines, saplings, shrubs, ferns, grass and rocks from ../../scenery are
strewn over the ground (the "scenery" of forest_parts()).

The road is a smooth line through the points of ROAD (world x, y in m, z
down, so +y is to the right of +x). Its elevation follows the hills,
smoothed along it so that its grade changes gently, with the crests of
JUMPS on it, and it falls to the outside through the off-camber corners of
CAMBERS; the ground around it is cut or filled to meet it over the banks.
The start is at the origin, heading +x, where the road is level at z = 0.

A model drives the stage with the root keys of forest_parts(), its frame,
terrain, surface materials, environment and scenery, and driver(...) among
its controllers, a driver along the middle of the road: import this script,
as examples/forest/make_polo_r_wrc_forest.py does.
"""

import os

import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter1d
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
SCENERY = os.path.join(HERE, "..", "..", "scenery")

SPACING = 0.5                      # m, of the elevation map
X0, Y0 = -80.0, -150.0             # world x, y of the terrain frame's origin
COLUMNS, ROWS = 801, 601           # 400 m along x, 300 m along y
MATERIAL_SPACING = 0.25            # m, of the material map

# the middle of the road, from the start: a straight with a jump, a fast
# right over a crest, a hairpin left, a long downhill right, an S and the finish
ROAD = [(-30.0, 0.0), (0.0, 0.0), (40.0, 0.0), (80.0, 6.0), (112.0, 26.0),
        (130.0, 58.0), (128.0, 92.0), (148.0, 118.0), (185.0, 118.0),
        (212.0, 92.0), (218.0, 55.0), (240.0, 22.0), (275.0, 8.0),
        (296.0, -22.0), (290.0, -62.0), (258.0, -90.0), (216.0, -100.0),
        (178.0, -82.0), (146.0, -62.0), (108.0, -70.0), (72.0, -96.0),
        (40.0, -110.0), (0.0, -112.0)]
ROAD_HALF_WIDTH = 3.4      # m, of the gravel
WHEEL_LINE = 0.78          # m, from the middle to each wheel line
SHOULDER = 1.0             # m beyond the gravel, falling to the ditch
DITCH = (5.6, 0.35)        # m from the middle, and depth
BANK = (6.6, 16.0)         # m: the ground meets the natural slope over these
START_FLAT = 40.0          # m of level road around the start
CROWN = 0.10               # m, the middle of the road above its edges
# the jumps: crests on the road, each at a station (m from the first point
# of ROAD), of a height (m), rising over a length (m) along the road and
# falling over another, each half a cosine wave. Over the top of a crest a
# car would need more than g to follow the road, height (pi / rise)^2 above
# g / v^2, so it leaves it; the long fall is the landing, sloping away under
# the car as it comes down. The one on the straight after the start is met
# at about 20 m/s, the fastest of the stage. The second, at the turn in of
# the long left at the top of the climb, which tightens to a radius of 37 m,
# lifts the car as it brakes into it: it comes down light on an off-camber
# road (CAMBERS) and slides (about 15 degrees at 17 m/s for the Polo of
# examples/forest)
JUMPS = [(72.0, 1.0, 10.0, 24.0), (578.0, 0.6, 10.0, 14.0)]
# off-camber corners: from a station to another (m), the road falls to the
# right by a slope (m/m; negative to the left), blended in over 10 m at each
# end
CAMBERS = [(565.0, 615.0, 0.08)]
# run-offs: from a station to another (m), the gravel widened by a width (m)
# to the right (negative: to the left), blended in over 15 m at each end, the
# shoulder, ditch and banks moved out with it; the one on the outside of the
# off-camber left keeps a car that slides wide there on the gravel
RUN_OFFS = [(560.0, 650.0, 5.0)]
# the crown under the wheels of a car on the middle, 1.5 m across
CROWN_AT_WHEELS = CROWN * (1.0 - (0.75 / ROAD_HALF_WIDTH) ** 2)


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def value_noise(x, y, scale, seed):
    """smooth noise from -1 to 1 with features about scale (m) across, from
    a fixed seed so that the stage is the same every time"""
    rng = np.random.default_rng(seed)
    grid = rng.uniform(-1.0, 1.0, (128, 128))
    u, v = x / scale, y / scale
    i, j = np.floor(u).astype(int), np.floor(v).astype(int)
    fu, fv = u - i, v - j
    fu, fv = fu * fu * (3 - 2 * fu), fv * fv * (3 - 2 * fv)
    g = lambda a, b: grid[a % 128, b % 128]
    return ((g(i, j) * (1 - fu) + g(i + 1, j) * fu) * (1 - fv) +
            (g(i, j + 1) * (1 - fu) + g(i + 1, j + 1) * fu) * fv)


def hills(x, y):
    """height of the natural ground (m, upward) at world x, y"""
    h = 7.0 * value_noise(x, y, 110.0, 1) + 3.0 * value_noise(x, y, 45.0, 2)
    h += 0.8 * value_noise(x, y, 14.0, 3) + 0.25 * value_noise(x, y, 5.0, 4)
    # rising to the north east, so that the road climbs and falls
    return h + 0.03 * (x - 100.0) - 0.02 * y


def centerline(step=0.25):
    """points of the middle of the road every step (m): a centripetal
    Catmull-Rom spline through ROAD; and the station of each"""
    p = np.array(ROAD)
    p = np.vstack([2 * p[0] - p[1], p, 2 * p[-1] - p[-2]])
    out = []
    for k in range(1, len(p) - 2):
        p0, p1, p2, p3 = p[k - 1], p[k], p[k + 1], p[k + 2]
        t0 = 0.0
        t1 = t0 + np.linalg.norm(p1 - p0) ** 0.5
        t2 = t1 + np.linalg.norm(p2 - p1) ** 0.5
        t3 = t2 + np.linalg.norm(p3 - p2) ** 0.5
        n = max(2, int(np.linalg.norm(p2 - p1) / 0.05))
        for t in np.linspace(t1, t2, n, endpoint=False):
            a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
            a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
            a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
            b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
            b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3
            out.append((t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2)
    out.append(p[-2])
    dense = np.array(out)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(dense, axis=0),
                                                         axis=1))])
    stations = np.arange(0.0, s[-1], step)
    xy = np.column_stack([np.interp(stations, s, dense[:, 0]),
                          np.interp(stations, s, dense[:, 1])])
    return xy, stations


def road_profile(xy, stations):
    """height of the middle of the road (m, upward) at each station: the
    hills under it, smoothed along it, level around the start and at 0
    there, and the crests of JUMPS"""
    h = hills(xy[:, 0], xy[:, 1])
    step = stations[1] - stations[0]
    h = gaussian_filter1d(h, 18.0 / step, mode="nearest")
    start = np.argmin(np.hypot(xy[:, 0], xy[:, 1]))
    h -= h[start]
    flat = smoothstep(START_FLAT, START_FLAT + 40.0,
                      np.abs(stations - stations[start]))
    h = h * flat
    for station, height, rise, fall in JUMPS:
        along = stations - station
        half = np.where(along < 0.0, rise, fall)
        h += np.where(np.abs(along) < half,
                      0.5 * height * (1.0 + np.cos(np.pi * along / half)),
                      0.0)
    return h


def road_frame(x, y, xy, stations):
    """for points x, y: the distance to the right of the middle of the road,
    the station of the nearest point of it, and its index"""
    tree = cKDTree(xy)
    distance, index = tree.query(np.column_stack([x.ravel(), y.ravel()]))
    tangent = np.gradient(xy, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    t = tangent[index]
    dx = x.ravel() - xy[index, 0]
    dy = y.ravel() - xy[index, 1]
    # with z down, the right of the tangent (tx, ty) is (-ty, tx)
    lateral = -dx * t[:, 1] + dy * t[:, 0]
    return (lateral.reshape(x.shape), stations[index].reshape(x.shape),
            index.reshape(x.shape))


def road_distance(lateral, station):
    """the distance from the middle of the road (m) at which the cross
    section is taken, for a point lateral (m) to the right of it at station:
    |lateral|, less the width of the run-offs (RUN_OFFS) beyond the crown of
    the gravel on their side, so that the edge of the gravel, the shoulder,
    the ditch and the banks are moved out by it"""
    d = np.abs(lateral)
    inner = ROAD_HALF_WIDTH - 0.6
    for first, last, width in RUN_OFFS:
        ramp = smoothstep(first - 15.0, first, station) * \
            (1.0 - smoothstep(last, last + 15.0, station))
        side = np.sign(lateral) == np.sign(width)
        w = np.where(side, abs(width) * ramp, 0.0)
        d = np.where(d > inner, inner + np.maximum(d - inner - w, 0.0), d)
    return d


def grid(spacing, columns, rows):
    x = X0 + spacing * np.arange(columns)
    y = Y0 + spacing * np.arange(rows)
    return np.meshgrid(x, y)  # rows along y, columns along x


def elevation(x, y, xy, stations, profile):
    """height of the ground (m, upward) at world x, y, with the road; and
    the distance from the middle of the road at which its cross section is
    taken (road_distance)"""
    lateral, station, index = road_frame(x, y, xy, stations)
    d = road_distance(lateral, station)
    natural = hills(x, y)
    road = profile[index]
    # the cross section: a crown on the gravel, the shoulder falling to the
    # ditch, and up the far side of the ditch
    half = ROAD_HALF_WIDTH
    crown = CROWN * (1.0 - np.minimum(d / half, 1.0) ** 2)
    shoulder = -0.06 * smoothstep(half, half + SHOULDER, d)
    ditch = -DITCH[1] * np.exp(-((d - DITCH[0]) / 0.7) ** 2)
    # level with z = 0 under the wheels at the start, where the car stands
    section = road + crown - CROWN_AT_WHEELS + shoulder + ditch
    for first, last, slope in CAMBERS:
        ramp = smoothstep(first - 10.0, first, station) * \
            (1.0 - smoothstep(last, last + 10.0, station))
        section -= ramp * slope * np.clip(lateral, -BANK[0], BANK[0])
    # the bumps of a gravel road, short waves at angles to each other, on
    # the gravel only
    bumps = (0.015 * np.sin(2 * np.pi * station / 6.3 + 0.3 * lateral) +
             0.010 * np.sin(2 * np.pi * (0.8 * station + 0.6 * lateral) / 3.7)
             + 0.012 * value_noise(x, y, 1.2, 5))
    start = np.argmin(np.hypot(xy[:, 0], xy[:, 1]))
    calm = smoothstep(15.0, 35.0, np.abs(station - stations[start]))
    section += calm * bumps * (1.0 - smoothstep(half - 0.5, half + 0.3, d))
    # the banks: cut or fill from the road to the natural ground
    blend = smoothstep(BANK[0], BANK[1], d)
    return section * (1.0 - blend) + natural * blend, d


def write_maps():
    xy, stations = centerline()
    profile = road_profile(xy, stations)
    x, y = grid(SPACING, COLUMNS, ROWS)
    h, _ = elevation(x, y, xy, stations, profile)
    Image.fromarray((-h).astype(np.float32), mode="F").save(
        os.path.join(HERE, "forest.tiff"), compression="tiff_deflate")

    columns = int(round((COLUMNS - 1) * SPACING / MATERIAL_SPACING)) + 1
    rows = int(round((ROWS - 1) * SPACING / MATERIAL_SPACING)) + 1
    x, y = grid(MATERIAL_SPACING, columns, rows)
    h, d = elevation(x, y, xy, stations, profile)
    material = np.zeros(h.shape, np.uint8)  # forest floor
    # the grass of the verges, ragged at the edge of the forest
    verge = d < 7.5 + 2.5 * value_noise(x, y, 4.0, 11) + \
        1.0 * value_noise(x, y, 1.3, 12)
    material[verge] = 3
    # bare rock in patches on the steep banks of the cuttings, where the
    # road is cut more than a meter into the hill
    gy, gx = np.gradient(h, MATERIAL_SPACING)
    slope = np.hypot(gx, gy)
    _, _, index = road_frame(x, y, xy, stations)
    cut = hills(x, y) - profile[index]
    rock = (slope > 0.3 + 0.1 * value_noise(x, y, 3.0, 13)) & \
        (cut > 1.0) & (d > BANK[0] + 0.5) & \
        (value_noise(x, y, 5.0, 19) > -0.3)
    material[rock] = 4
    # the gravel, its edge wandering a little
    edge = ROAD_HALF_WIDTH + 0.25 * value_noise(x, y, 2.0, 14) + \
        0.12 * value_noise(x, y, 0.6, 15)
    material[d < edge] = 1
    # the packed wheel lines, where the cars have swept the gravel aside
    lines = (np.abs(d - WHEEL_LINE) < 0.25 + 0.1 * value_noise(x, y, 1.5, 16))
    material[lines & (d < edge - 0.3)] = 2
    middle = d < 0.25 + 0.1 * value_noise(x, y, 1.0, 17)
    material[middle & (value_noise(x, y, 6.0, 18) > 0.3)] = 2
    Image.fromarray(material, mode="L").save(
        os.path.join(HERE, "forest_materials.tiff"),
        compression="tiff_deflate")
    return xy, stations


def driven_path(step=2.0):
    """the middle of the road every step (m), [x0, y0, x1, y1, ...]"""
    xy, stations = centerline(step)
    return [round(float(v), 3) for v in xy.ravel()]


def relative(path, model_dir):
    """path from the directory of the model file, ending in /"""
    return os.path.relpath(path, model_dir).replace(os.sep, "/") + "/"


def forest_parts(model_dir):
    """the root keys of the stage: its frame, terrain, surface materials,
    environment and scenery, their files named from model_dir, the
    directory of the model file"""
    include = relative(HERE, model_dir)
    textures = include + "textures/"
    scenery = relative(SCENERY, model_dir)
    material_map = {
        "filename": include + "forest_materials.tiff",
        "x_scale": MATERIAL_SPACING, "y_scale": MATERIAL_SPACING,
        "blend_width": 0.25,
        "materials": [{"value": 0, "material": "forest_floor"},
                      {"value": 1, "material": "gravel"},
                      {"value": 2, "material": "wheel_lines"},
                      {"value": 3, "material": "verge"},
                      {"value": 4, "material": "rock"}]}

    def texture_set(name, scale, roughness, height=False, **extra):
        material = {
            "texture_filename": textures + name + "_diff.jpg",
            "normal_map_filename": textures + name + "_nor_gl.jpg",
            "texture_scale": scale, "roughness": roughness}
        if height:
            material["height_map_filename"] = textures + name + "_disp.jpg"
        material.update(extra)
        return material

    def meshes(names):
        return [{"type": "mesh", "filename": [scenery + n + ".glb"]}
                for n in names]

    return {
        "frames": [{"name": "forest", "T": [X0, Y0, 0.0]}],
        "environment": {
            "sky": {"type": "physical", "turbidity": 3.0},
            # a late morning sun, low enough for long shadows in the trees
            "sun": {"azimuth": 140.0, "elevation": 35.0},
            "fog": {"density": 0.0025, "height_falloff": 0.05},
        },
        "surface_materials": [
            {"name": "forest_floor", "label": "ground",
             **texture_set("forest_leaves_02", 2.5, 0.9,
                           color=[0.42, 0.46, 0.34]),
             "soil": {"name": "sandy_loam"}},
            {"name": "gravel", "label": "gravel",
             # stones a few centimeters proud of the dirt
             **texture_set("rocky_trail", 2.0, 0.85, height=True,
                           height_scale=0.06),
             "soil": {"name": "sandy_loam"}},
            {"name": "wheel_lines", "label": "gravel",
             **texture_set("rocky_trail", 2.0, 0.8, normal_scale=0.4,
                           color=[0.78, 0.74, 0.70]),
             "soil": {"name": "sandy_loam"}},
            {"name": "verge", "label": "grass",
             **texture_set("sparse_grass", 2.0, 0.9,
                           color=[0.9, 0.95, 0.8]),
             "soil": {"name": "sandy_loam"}},
            {"name": "rock", "label": "rock",
             **texture_set("mossy_rock", 3.0, 0.75, height=True,
                           height_scale=0.1),
             "soil": {"name": "sandy_loam"}},
        ],
        "terrain": [
            {"type": "basic", "name": "forest_stage",
             "parent": [["fixed"], "forest"],
             "filename": include + "forest.tiff",
             "x_scale": SPACING, "y_scale": SPACING, "z_scale": 1.0,
             "coordinate_type": "sae", "interpolation_type": "bicubic",
             "texture_variation": True, "material_map": material_map}],
        "scenery": [
            {"name": "trees", "terrain": "forest_stage",
             "materials": ["forest_floor"], "margin": 2.0,
             "density": 0.06, "spacing": 3.5, "scale": [0.75, 1.15],
             "depth": 0.25, "seed": 1, "draw_distance": 250.0,
             "geometry": meshes(["fir_a", "fir_b", "fir_c",
                                 "pine_a", "pine_b", "pine_c"])},
            {"name": "saplings", "terrain": "forest_stage",
             "materials": ["forest_floor", "verge"], "margin": 0.8,
             "density": 0.012, "spacing": 2.5, "avoid": ["trees"],
             "scale": [1.2, 2.6], "depth": 0.05, "seed": 2,
             "draw_distance": 120.0,
             "geometry": meshes(["fir_sapling_a", "fir_sapling_b",
                                 "fir_sapling_c", "pine_sapling_a",
                                 "pine_sapling_b", "pine_sapling_c"])},
            {"name": "shrubs", "terrain": "forest_stage",
             "materials": ["forest_floor", "verge"], "margin": 0.5,
             "density": 0.04, "spacing": 1.6, "avoid": ["trees", "saplings"],
             "scale": [0.6, 1.1], "tilt": 0.3, "seed": 3,
             "draw_distance": 80.0,
             "geometry": meshes(["shrub_02_a", "shrub_02_b", "shrub_02_c",
                                 "shrub_02_d", "fern_a", "fern_b"])},
            {"name": "verge_grass", "terrain": "forest_stage",
             "materials": ["verge"], "margin": 0.2,
             "density": 6.0, "spacing": 0.3, "scale": [1.4, 2.4],
             "tilt": 1.0, "seed": 4, "draw_distance": 45.0,
             "geometry": meshes(["grass_a", "grass_b", "grass_c", "grass_d",
                                 "grass_e"])},
            {"name": "forest_grass", "terrain": "forest_stage",
             "materials": ["forest_floor"], "density": 0.6,
             "spacing": 0.7, "avoid": ["trees"], "scale": [0.8, 1.5],
             "tilt": 1.0, "seed": 5, "draw_distance": 35.0,
             "geometry": meshes(["grass_a", "grass_c", "grass_e"])},
            {"name": "rocks", "terrain": "forest_stage",
             "materials": ["forest_floor", "rock"], "margin": 0.5,
             "density": 0.004, "spacing": 3.0, "avoid": ["trees"],
             "scale": [0.4, 0.9], "tilt": 1.0, "depth": 0.25, "seed": 6,
             "draw_distance": 150.0,
             "geometry": meshes(["mossy_rock_a", "mossy_rock_b",
                                 "mossy_rock_c", "mossy_rock_d",
                                 "mossy_rock_e", "mossy_rock_f", "boulder",
                                 "stump", "dead_trunk"])},
        ],
    }


def driver(body_frame_pair, wheelbase, **extra):
    """a driver controller that follows the middle of the road from the
    start to the finish"""
    return {"type": "driver", "name": "driver",
            "body_frame_pair": body_frame_pair, "wheelbase": wheelbase,
            "path": driven_path(), **extra}


if __name__ == "__main__":
    xy, stations = write_maps()
    print("wrote forest.tiff and forest_materials.tiff in %s; the road is "
          "%.0f m long" % (HERE, stations[-1]))
