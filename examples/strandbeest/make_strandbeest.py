#!/usr/bin/env python3
"""Write strandbeest.json: a walker of twelve Jansen legs on one crankshaft,
with bone-like links.

The leg is Theo Jansen's linkage, with his link lengths (the "holy numbers",
in centimeters): the crank turns about O, and the rest of the leg hangs from
a fixed pivot Z, a to the rear of O and l below it. In the plane of a leg
(u forward, v up), with A the crank pin:

* link j joins A to C, and link k joins A to F;
* the upper triangle Z, C, D (sides b, d, e) turns about Z;
* link c joins Z to F;
* link f joins D to E;
* the lower triangle F, E, G (sides g, i, h) carries the foot G.

Each leg has six links and nine revolute joints besides the crank: three
loops, which leave the leg no freedom of its own, so the crank drives it.

The walker follows shift's axes (x forward, y to the right, z down):

* the crankshaft is along y, with the chassis origin on its axis. Six
  stations along it each carry a pair of legs on the same crank pin, a front
  leg and its mirror image across the y-z plane, a rear leg (Jansen's pairs).
  A rear leg is not a mirrored mesh: its plane has u backward and its normal
  w along -y, so its frames stay right handed and it uses the same meshes;
* the crank pins of the three left stations are 120 degrees apart, those of
  the right ones too, and the right ones are 180 degrees from the left ones;
* the links of a leg are in four layers along w, outward from the station
  plane: j; k; the two triangles; c and f. All the joints of a leg are in the
  station plane, the meshes are offset to their layers;
* a single PID controller drives the crankshaft to a speed, from a speed
  sensor;
* only the feet take part in contact: a sphere at G of each lower triangle,
  on the ground plane.

The initial joint angles are those of the linkage drawn with circle
intersections at the initial crank angles; shift closes the loops from them.

The links are bones, surfaces of revolution with knobs (condyles) at the
pins and a little irregularity, textured by triplanar projection with the
maps of make_bone_textures.py. The ground is a sand beach.

usage: make_strandbeest.py
"""

import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# the helpers that write meshes are shared with the other generators
sys.path.insert(0, os.path.join(HERE, ".."))
import mesh_builder  # noqa: E402

# Jansen's holy numbers, in centimeters
HOLY = {"a": 38.0, "b": 41.5, "c": 39.3, "d": 40.1, "e": 55.8, "f": 39.4,
        "g": 36.7, "h": 65.7, "i": 49.0, "j": 50.0, "k": 61.9, "l": 7.8,
        "m": 15.0}
SCALE = 0.01                        # meters per centimeter
L = {key: value * SCALE for key, value in HOLY.items()}

STATION_Y = [-0.95, -0.57, -0.19, 0.19, 0.57, 0.95]   # left to right
STATION_PHASE = [0.0, 120.0, 240.0, 180.0, 300.0, 60.0]   # degrees
CRANK_ANGLE_0 = 0.0                 # of the crank pin of phase 0, from +u
LAYER_W0 = 0.035                    # of the first layer, from the station
LAYER_DW = 0.045                    # between layers
LAYER = {"j": 0, "k": 1, "upper": 2, "lower": 2, "c": 3, "f": 3}
END_Y = 1.25                        # of the ends of the chassis and shaft

BONE_R = 0.011                      # radius of a link at its middle
KNOB_R = 0.022                      # of the knobs at the pins
FOOT_R = 0.03                       # of the foot, and its contact sphere
LINK_LINE_DENSITY = 1.5             # kg per meter of link
CHASSIS_MASS = 40.0
SHAFT_MASS = 6.0
KP = 1500.0                         # of the PID on the crankshaft
KI = 3000.0
CRANK_SPEED = -2.0                  # rad/s about +w of the front legs (+y)
GROUND_CLEARANCE = 0.01             # of the lowest foot at the start
TEXTURE_SCALE = 0.15                # of one tile of the bone texture, m

BONE_MTL = """# bone: the color, roughness and normal maps of make_bone_textures.py
newmtl bone
Kd 1.0 1.0 1.0
Pm 0.0
Pr 1.0
map_Kd bone_base_color.png
map_Pr bone_roughness.png
norm -bm 0.8 bone_normal.png
"""


# ---------------------------------------------------------------------------
# the linkage in the plane of a leg

def circles(p, rp, q, rq):
    """the two intersections of the circle of radius rp about p and that of
    radius rq about q"""
    dx, dy = q[0] - p[0], q[1] - p[1]
    dist = math.hypot(dx, dy)
    along = (rp * rp - rq * rq + dist * dist) / (2.0 * dist)
    across = math.sqrt(max(rp * rp - along * along, 0.0))
    mx, my = p[0] + along * dx / dist, p[1] + along * dy / dist
    return ((mx - across * dy / dist, my + across * dx / dist),
            (mx + across * dy / dist, my - across * dx / dist))


def leg_points(crank_angle):
    """the pins of a leg in its plane (u forward, v up), for the crank pin at
    crank_angle from +u"""
    O = (0.0, 0.0)
    Z = (-L["a"], -L["l"])
    A = (L["m"] * math.cos(crank_angle), L["m"] * math.sin(crank_angle))
    C = max(circles(A, L["j"], Z, L["b"]), key=lambda p: p[1])     # above
    F = min(circles(A, L["k"], Z, L["c"]), key=lambda p: p[1])     # below
    D = min(circles(Z, L["d"], C, L["e"]), key=lambda p: p[0])     # behind
    E = min(circles(D, L["f"], F, L["g"]), key=lambda p: p[0])     # behind
    G = min(circles(E, L["h"], F, L["i"]), key=lambda p: p[1])     # below
    return {"O": O, "Z": Z, "A": A, "C": C, "D": D, "E": E, "F": F, "G": G}


def angle(p, q):
    return math.atan2(q[1] - p[1], q[0] - p[0])


def local(origin, theta, p):
    """p in the axes of a body with its origin at origin, turned by theta"""
    dx, dy = p[0] - origin[0], p[1] - origin[1]
    c, s = math.cos(theta), math.sin(theta)
    return (c * dx + s * dy, -s * dx + c * dy)


# the links: name, the pin at the origin, the pin along x, more pins, and the
# edges (bones) between pins
LINKS = {
    "j": ("A", "C", [], [("A", "C")]),
    "k": ("A", "F", [], [("A", "F")]),
    "upper": ("Z", "C", ["D"], [("Z", "C"), ("C", "D"), ("D", "Z")]),
    "c": ("Z", "F", [], [("Z", "F")]),
    "f": ("D", "E", [], [("D", "E")]),
    "lower": ("F", "E", ["G"], [("F", "E"), ("E", "G"), ("G", "F")]),
}


def link_geometry(points):
    """the angle of each link in the plane of the leg and its pins in its
    own axes"""
    result = {}
    for name, (origin, along, more, _) in LINKS.items():
        theta = angle(points[origin], points[along])
        pins = {p: local(points[origin], theta, points[p])
                for p in [origin, along] + more}
        result[name] = (theta, pins)
    return result


# the reference drawing of the links: their shapes in their own axes do not
# depend on the crank angle
REFERENCE = link_geometry(leg_points(0.0))


# ---------------------------------------------------------------------------
# meshes

def bone(mesh, material, p, q, side, radius, knob_p, knob_q, seed):
    """a bone from p to q: a slender shaft, thicker toward its ends, with a
    knob at each end, and a little irregularity; side is a unit vector across
    the bone, the axis of its two-lobed knobs"""
    axis = [q[i] - p[i] for i in range(3)]
    length = math.sqrt(sum(x * x for x in axis))
    e1 = [x / length for x in axis]
    dot = sum(side[i] * e1[i] for i in range(3))
    e2 = [side[i] - dot * e1[i] for i in range(3)]
    n2 = math.sqrt(sum(x * x for x in e2))
    e2 = [x / n2 for x in e2]
    e3 = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2],
          e1[0] * e2[1] - e1[1] * e2[0]]
    # the irregularity: a few waves along and around the bone, with phases
    # and amplitudes from the seed (a fixed sequence, no randomness)
    waves = []
    for k in range(5):
        h = math.sin(seed * 12.9898 + k * 78.233) * 43758.5453
        h -= math.floor(h)
        waves.append((2.0 + 3.0 * k + 4.0 * h, (k % 3), 6.2832 * h,
                      0.02 / (1 + k)))

    def profile(x, t):
        u = min(max(x / length, 0.0), 1.0)
        shaft = radius * (1.0 + 0.6 * (2.0 * u - 1.0) ** 4) \
            if 0.0 <= x <= length else 0.0
        lobes_p = 1.0 + 0.10 * math.cos(2.0 * t)
        lobes_q = 1.0 + 0.10 * math.cos(2.0 * t + 0.7)
        kp = math.sqrt(max(knob_p * knob_p - x * x, 0.0)) * lobes_p
        kq = math.sqrt(max(knob_q * knob_q - (x - length) ** 2, 0.0)) * lobes_q
        r = (shaft ** 6 + kp ** 6 + kq ** 6) ** (1.0 / 6.0)
        wobble = sum(a * math.sin(f * math.pi * u + n * t + ph)
                     for f, n, ph, a in waves)
        return r * (1.0 + wobble)

    # stations along the bone: dense at the poles of the knobs
    xs = [-knob_p * math.cos(0.5 * math.pi * s / 10) for s in range(10)]
    xs += [length * s / 28 for s in range(29)]
    xs += [length + knob_q * math.sin(0.5 * math.pi * s / 10)
           for s in range(1, 11)]
    sides = 22
    grid = []
    for x in xs:
        ring = []
        for k in range(sides):
            t = 2.0 * math.pi * k / sides
            r = profile(x, t)
            ring.append([p[i] + x * e1[i] +
                         r * (math.cos(t) * e2[i] + math.sin(t) * e3[i])
                         for i in range(3)])
        grid.append(ring)

    def sub(a, b):
        return [a[i] - b[i] for i in range(3)]

    def normal(i, k):
        if i == 0:
            return [-x for x in e1]
        if i == len(xs) - 1:
            return list(e1)
        along = sub(grid[i + 1][k], grid[i - 1][k])
        around = sub(grid[i][(k + 1) % sides], grid[i][k - 1])
        n = [around[1] * along[2] - around[2] * along[1],
             around[2] * along[0] - around[0] * along[2],
             around[0] * along[1] - around[1] * along[0]]
        # outward: away from the axis
        radial = sub(grid[i][k], [p[j] + xs[i] * e1[j] for j in range(3)])
        if sum(n[j] * radial[j] for j in range(3)) < 0.0:
            n = [-x for x in n]
        return n

    normals = [[normal(i, k) for k in range(sides)] for i in range(len(xs))]
    for i in range(len(xs) - 1):
        for k in range(sides):
            k1 = (k + 1) % sides
            a, b = (i, k), (i, k1)
            c, d = (i + 1, k1), (i + 1, k)
            for tri in ((a, b, c), (a, c, d)):
                mesh.triangle(material,
                              [grid[m][n] for m, n in tri],
                              [normals[m][n] for m, n in tri])


def bone_mesh(name, bones):
    """writes name.obj and name.mtl, bones a list of (p, q, side, radius,
    knob_p, knob_q)"""
    mesh = mesh_builder.Mesh(name, HERE)
    mesh.material("bone", (1.0, 1.0, 1.0))
    for seed, (p, q, side, radius, kp, kq) in enumerate(bones):
        bone(mesh, "bone", p, q, side, radius, kp, kq,
             seed + 7 * len(name))
    mesh.write()
    with open(os.path.join(HERE, name + ".mtl"), "w") as f:
        f.write(BONE_MTL)
    return [name + ".obj", name + ".mtl"]


def mesh_geometry(files):
    return [{"type": "mesh", "texture_mapping": "triplanar",
             "texture_scale": TEXTURE_SCALE, "filename": files}]


def rod_inertia(segments, line_density):
    """mass, center of mass and inertia tensor about it of thin rods, each
    (p, q) in body axes"""
    mass = 0.0
    first = [0.0, 0.0, 0.0]
    for p, q in segments:
        m = line_density * math.dist(p, q)
        mass += m
        for i in range(3):
            first[i] += m * 0.5 * (p[i] + q[i])
    com = [x / mass for x in first]
    tensor = [[0.0] * 3 for _ in range(3)]
    for p, q in segments:
        length = math.dist(p, q)
        m = line_density * length
        d = [(q[i] - p[i]) / length for i in range(3)]
        c = [0.5 * (p[i] + q[i]) - com[i] for i in range(3)]
        cc = sum(x * x for x in c)
        for i in range(3):
            for k in range(3):
                delta = 1.0 if i == k else 0.0
                # the rod about its center, a thin tube of radius BONE_R
                tensor[i][k] += m * length ** 2 / 12.0 * (delta - d[i] * d[k])
                tensor[i][k] += m * BONE_R ** 2 / 2.0 * d[i] * d[k]
                # parallel axes
                tensor[i][k] += m * (cc * delta - c[i] * c[k])
    return mass, com, tensor


def inertia_entry(mass, tensor):
    return {"mass": round(mass, 6),
            "Ixx": round(tensor[0][0], 8), "Iyy": round(tensor[1][1], 8),
            "Izz": round(tensor[2][2], 8), "Ixy": round(tensor[0][1], 8),
            "Iyz": round(tensor[1][2], 8), "Izx": round(tensor[2][0], 8)}


# where two links that share a pin are not in adjacent layers (j and the
# upper triangle at C, with k between them), the link in the lower layer
# carries a pin across the gap: link name -> [(pin, layer it reaches)]
BRIDGES = {}
for pin_name in {p for _, (o, a, more, _) in LINKS.items()
                 for p in [o, a] + more}:
    layers = sorted(LAYER[name] for name, (o, a, more, _) in LINKS.items()
                    if pin_name in [o, a] + more)
    for lo, hi in zip(layers, layers[1:]):
        if hi - lo > 1:
            lower = next(name for name, (o, a, more, _) in LINKS.items()
                         if LAYER[name] == lo and pin_name in [o, a] + more)
            BRIDGES.setdefault(lower, []).append((pin_name, hi))
PIN_R = 0.012                       # of a pin across a layer

# the links, each in its layer along w (z of the body)
link_files = {}
link_mass = {}
for name, (origin, along, more, edges) in LINKS.items():
    theta, pins = REFERENCE[name]
    w = LAYER_W0 + LAYER[name] * LAYER_DW
    p3 = {key: (x, y, w) for key, (x, y) in pins.items()}
    bones = []
    for a, b in edges:
        bones.append((p3[a], p3[b], (0.0, 0.0, 1.0), BONE_R,
                      FOOT_R if a == "G" else KNOB_R,
                      FOOT_R if b == "G" else KNOB_R))
    for pin_name, layer in BRIDGES.get(name, []):
        x, y, _ = p3[pin_name]
        bones.append(((x, y, w), (x, y, LAYER_W0 + layer * LAYER_DW),
                      (1.0, 0.0, 0.0), PIN_R, 0.016, 0.016))
    link_files[name] = bone_mesh(f"strandbeest_{name}", bones)
    link_mass[name] = rod_inertia([(p3[a], p3[b]) for a, b in edges],
                                  LINK_LINE_DENSITY)

# the crank pins of the stations, in the axes of the crankshaft (those of a
# front leg: u forward, v up, w along +y) at the start
crank_angles = [CRANK_ANGLE_0 + math.radians(phase) for phase in
                STATION_PHASE]
pins = [(L["m"] * math.cos(a), L["m"] * math.sin(a), y)
        for a, y in zip(crank_angles, STATION_Y)]
# the crankshaft is bent at each station, like a crank throw: links j and k
# (the inner two layers of both legs on the pin) sweep across the axis of the
# shaft, so the axle stops at a crank web on either side of them, in the
# layer of the triangles (which keep 0.24 m from the throw), and the crank
# pin runs between the two webs
web_w = LAYER_W0 + LAYER["upper"] * LAYER_DW
shaft_bones = []
axle_ends = [-END_Y]
for x, y, w in pins:
    axle_ends += [w - web_w, w + web_w]
    for side in (-1.0, 1.0):
        shaft_bones.append(((0.0, 0.0, w + side * web_w),
                            (x, y, w + side * web_w), (0.0, 0.0, 1.0),
                            0.016, 0.03, 0.026))
    shaft_bones.append(((x, y, w - web_w), (x, y, w + web_w),
                        (x, y, 0.0), 0.012, 0.016, 0.016))
axle_ends.append(END_Y)
for w0, w1 in zip(axle_ends[0::2], axle_ends[1::2]):
    shaft_bones.append(((0.0, 0.0, w0), (0.0, 0.0, w1), (1.0, 0.0, 0.0),
                        0.018, 0.03, 0.03))
shaft_files = bone_mesh("strandbeest_crankshaft", shaft_bones)

# the chassis, a rib cage of bones in its own axes (x forward, z down): the
# two axles of the fixed pivots Z, the front legs' to the rear of the crank,
# the rear legs' to the front, a spine above the crankshaft, and at each end
# a frame from the pivots to the bearing of the crankshaft and to the spine
PIVOT = {"front": (-L["a"], L["l"]), "rear": (L["a"], L["l"])}   # (x, z)
SPINE_Z = -0.32
chassis_bones = []
for x, z in PIVOT.values():
    chassis_bones.append(((x, -END_Y, z), (x, END_Y, z), (1.0, 0.0, 0.0),
                          0.016, 0.03, 0.03))
chassis_bones.append(((0.0, -END_Y, SPINE_Z), (0.0, END_Y, SPINE_Z),
                      (1.0, 0.0, 0.0), 0.02, 0.04, 0.04))
for y in (-END_Y, END_Y):
    bearing = (0.0, y, 0.0)
    top = (0.0, y, SPINE_Z)
    for x, z in PIVOT.values():
        chassis_bones.append(((x, y, z), bearing, (0.0, 1.0, 0.0), 0.014,
                              0.03, 0.035))
        chassis_bones.append(((x, y, z), top, (0.0, 1.0, 0.0), 0.014, 0.03,
                              0.04))
    chassis_bones.append((bearing, top, (0.0, 1.0, 0.0), 0.014, 0.035, 0.04))
chassis_files = bone_mesh("strandbeest_chassis", chassis_bones)

# the beach: a square of sand under the walker, for show, 3 mm above the
# ground plane so that it covers the viewer's floor there (the feet press
# into the ground a few millimeters)
BEACH = 8.0
BEACH_Z = -0.003
beach = mesh_builder.Mesh("strandbeest_beach", HERE)
beach.material("sand", (1.0, 1.0, 1.0), rough=1.0)
beach.flat("sand", [(-BEACH, -BEACH, BEACH_Z), (-BEACH, BEACH, BEACH_Z),
                    (BEACH, BEACH, BEACH_Z), (BEACH, -BEACH, BEACH_Z)])
beach.write()
with open(os.path.join(HERE, "strandbeest_beach.mtl"), "w") as f:
    f.write("# sand of the viewer's textures\nnewmtl sand\nKd 1.0 1.0 1.0\n"
            "Pm 0.0\nPr 0.9\nmap_Kd qrc:/textures/dry_sand.jpg\n"
            "norm -bm 0.6 qrc:/textures/dry_sand_normal.jpg\n")


# ---------------------------------------------------------------------------
# the model

# the axes of the planes of the legs in the chassis axes, by rows: a front
# leg has u along x, v along -z and w along y; a rear leg u along -x, v
# along -z and w along -y
TURN = {"front": [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
        "rear": [[-1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, -1.0, 0.0]]}
# a rear plane in the axes of a front one (those of the crankshaft)
REAR_IN_FRONT = [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]]

chassis_frames = [{"name": "crankshaft", "T": [0.0, 0.0, 0.0],
                   "R": TURN["front"]}]
shaft_frames = []
bodies = []
joints = []
lowest_foot = 0.0
for s, (y, crank, pin) in enumerate(zip(STATION_Y, crank_angles, pins)):
    for leg in ("front", "rear"):
        prefix = f"s{s + 1}_{leg}"
        sign = 1.0 if leg == "front" else -1.0
        # the crank pin in the plane of this leg: a rear leg sees u reversed
        a_leg = crank if leg == "front" else math.pi - crank
        points = leg_points(a_leg)
        lowest_foot = min(lowest_foot, points["G"][1])
        geometry = link_geometry(points)
        theta = {name: geometry[name][0] for name in LINKS}
        # the triangles keep their shapes: the same branch of each circle
        # intersection as in the reference drawing
        for name in LINKS:
            for p, xy in geometry[name][1].items():
                ref = REFERENCE[name][1][p]
                assert math.dist(xy, ref) < 1e-9, (prefix, name, p)
        # the fixed pivot Z on the chassis, the crank pin A on the shaft
        zx, zz = PIVOT[leg]
        chassis_frames.append({"name": f"{prefix}_Z", "T": [zx, y, zz],
                               "R": TURN[leg]})
        shaft_frames.append({"name": f"{prefix}_A", "T": list(pin),
                             **({} if leg == "front" else
                                {"R": REAR_IN_FRONT})})
        for name, (origin, along, more, edges) in LINKS.items():
            _, pins_local = REFERENCE[name]
            mass, com, tensor = link_mass[name]
            bodies.append({
                "name": f"{prefix}_{name}",
                "geometry": mesh_geometry(link_files[name]),
                "frames": [{"name": p, "T": [round(pins_local[p][0], 6),
                                             round(pins_local[p][1], 6), 0.0]}
                           for p in [along] + more],
                "center_of_mass": [round(x, 6) for x in com],
                "inertia": inertia_entry(mass, tensor),
                **({"collision_geometry": [{
                    "name": f"{prefix}_foot", "type": "sphere",
                    "radius": FOOT_R,
                    "pose": {"T": [round(pins_local["G"][0], 6),
                                   round(pins_local["G"][1], 6),
                                   LAYER_W0 + LAYER["lower"] * LAYER_DW]},
                    "material": "foot"}]} if name == "lower" else {}),
            })

        def joint(name, parent, parent_frame, child, child_frame, value):
            joints.append({"name": f"{prefix}_{name}", "type": "revolute",
                           "parent": [[parent], parent_frame],
                           "child": [[child], child_frame],
                           "axis": [0, 0, 1],
                           "initial_displacement": round(value, 6),
                           "initial_velocity": 0.0})

        body = {name: f"{prefix}_{name}" for name in LINKS}
        joint("Z_upper", "chassis", f"{prefix}_Z", body["upper"], "origin",
              theta["upper"])
        joint("Z_c", "chassis", f"{prefix}_Z", body["c"], "origin",
              theta["c"])
        joint("A_j", "crankshaft", f"{prefix}_A", body["j"], "origin",
              theta["j"])
        joint("A_k", "crankshaft", f"{prefix}_A", body["k"], "origin",
              theta["k"])
        joint("C", body["j"], "C", body["upper"], "C",
              theta["upper"] - theta["j"])
        joint("F_k", body["k"], "F", body["c"], "F", theta["c"] - theta["k"])
        joint("F_lower", body["c"], "F", body["lower"], "origin",
              theta["lower"] - theta["c"])
        joint("D", body["upper"], "D", body["f"], "origin",
              theta["f"] - theta["upper"])
        joint("E", body["f"], "E", body["lower"], "E",
              theta["lower"] - theta["f"])

# the chassis floats; its origin is on the axis of the crankshaft, and the
# lowest foot starts just above the ground (z = 0, z points down)
start_z = -(-lowest_foot + FOOT_R + GROUND_CLEARANCE)
chassis = {
    "name": "chassis",
    "geometry": mesh_geometry(chassis_files),
    "frames": chassis_frames,
    "center_of_mass": [0.0, 0.0, 0.0],
    "inertia": {"mass": CHASSIS_MASS,
                "Ixx": CHASSIS_MASS * (4 * END_Y ** 2 + 0.4 ** 2) / 12,
                "Iyy": CHASSIS_MASS * (0.8 ** 2 + 0.4 ** 2) / 12,
                "Izz": CHASSIS_MASS * (4 * END_Y ** 2 + 0.8 ** 2) / 12,
                "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0},
}
crankshaft = {
    "name": "crankshaft",
    "geometry": mesh_geometry(shaft_files),
    "frames": shaft_frames,
    "center_of_mass": [0.0, 0.0, 0.0],
    "inertia": {"mass": SHAFT_MASS,
                "Ixx": SHAFT_MASS * END_Y ** 2 / 3,
                "Iyy": SHAFT_MASS * END_Y ** 2 / 3,
                "Izz": SHAFT_MASS * L["m"] ** 2 / 2,
                "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0},
}
beach_body = {
    "name": "beach",
    "geometry": [{"type": "mesh", "texture_mapping": "triplanar",
                  "texture_scale": 2.0,
                  "filename": ["strandbeest_beach.obj",
                               "strandbeest_beach.mtl"]}],
    "center_of_mass": [0.0, 0.0, 0.0],
    "inertia": {"mass": 1.0, "Ixx": 1.0, "Iyy": 1.0, "Izz": 1.0,
                "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0},
}
bodies = [chassis, crankshaft, beach_body] + bodies
joints = [
    {"name": "beach_fixed", "type": "rigid",
     "parent": [["fixed"], "origin"], "child": [["beach"], "origin"]},
    {"name": "chassis_float", "type": "float",
     "parent": [["fixed"], "origin"], "child": [["chassis"], "origin"],
     "initial_displacement": {"T": [0.0, 0.0, round(start_z, 6)]}},
    {"name": "crankshaft_bearing", "type": "revolute",
     "parent": [["chassis"], "crankshaft"],
     "child": [["crankshaft"], "origin"], "axis": [0, 0, 1],
     "initial_displacement": 0.0, "initial_velocity": 0.0},
] + joints

model = {
    "model_parameters": {
        "version": "0.2",
        "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                  "time": "seconds"},
        # z points down: gravity acts along +z
        "gravity": [0, 0, 9.81],
        "include_directories": ["."],
        "enable_contact_detection": True,
        "solvers": [{"type": "rnca"}],
        "integrators": [{"type": "ode113", "time_span": 12.0,
                         "time_step": 0.02, "relative_error": 1e-05,
                         "absolute_error": 1e-05}],
    },
    "frames": [{"name": "sun", "T": [4.0, -3.0, -8.0]},
               {"name": "fill", "T": [-6.0, 6.0, -5.0]}],
    "lights": [
        {"name": "sun", "type": "point", "parent": ["fixed", "sun"],
         "Ka": [0.12, 0.11, 0.10], "Kd": [0.85, 0.80, 0.70],
         "Ks": [0.3, 0.3, 0.3]},
        {"name": "fill", "type": "point", "parent": ["fixed", "fill"],
         "Ka": [0.05, 0.05, 0.06], "Kd": [0.35, 0.38, 0.45],
         "Ks": [0.1, 0.1, 0.1]},
    ],
    # the ground, a plane with its normal up (-z) through the origin
    "collision_geometry": [{"name": "ground", "type": "plane",
                            "normal": [0, 0, -1], "offset": 0.0,
                            "material": "sand"}],
    "assemblies": [{
        "name": "strandbeest",
        "bodies": bodies,
        "joints": joints,
        "sensors": [{"type": "joint", "name": "crankshaft_speed",
                     "joint": "crankshaft_bearing", "quantity": "velocity"}],
        "controllers": [{"type": "pid", "name": "crankshaft_drive",
                         "kp": KP, "ki": KI, "kd": 0.0,
                         "target": CRANK_SPEED, "sensor": "crankshaft_speed",
                         "joint": "crankshaft_bearing"}],
        "force_modules": [{
            "type": "contact", "name": "contact",
            # the feet on the sand: Hertz contact of a sphere on a plane
            "damping_depth": 0.001,
            "materials": [
                {"name": "foot", "youngs_modulus": 2e6, "poisson_ratio": 0.45,
                 "damping": 8000.0, "friction": 0.9},
                {"name": "sand", "youngs_modulus": 5e7,
                 "poisson_ratio": 0.3, "damping": 1e9, "friction": 0.8},
            ],
        }],
    }],
}

header = """  // A Strandbeest (written by make_strandbeest.py): twelve of Theo Jansen's
  // legs, with his link lengths, in six pairs on one crankshaft that a PID
  // controller drives, on a chassis that is free to move. The links are bones,
  // textured with the maps of make_bone_textures.py. Only the feet, spheres,
  // touch the ground plane."""
text = json.dumps(model, indent=2)
text = text.replace("{\n", "{\n" + header + "\n", 1)
with open(os.path.join(HERE, "strandbeest.json"), "w") as f:
    f.write(text + "\n")
print("wrote", os.path.join(HERE, "strandbeest.json"),
      f"({len(bodies)} bodies, {len(joints)} joints)")
