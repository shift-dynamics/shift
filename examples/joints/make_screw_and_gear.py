#!/usr/bin/env python3
"""Writes screw_and_gear.json and its meshes, to inspect the screw and gear
joints.

* a gear train of four spur gears with involute teeth on fixed axes. Each
  gear turns on a revolute joint to the board, and each pair in mesh is
  joined by a gear joint, which closes a loop: the gear joint makes the child
  roll on the parent without slipping, so the train has one degree of freedom,
  and the gears must mesh at every instant if the joints are right. The
  first gear is given a speed;
* two screws below it, one with a right-handed thread and one with a
  left-handed thread, each a threaded rod that turns and advances through a
  nut held in a bracket, on a screw joint. They have the same speed, and
  their pitches have opposite signs: the right-handed rod advances in the
  direction its thread leads, the left-handed one the other way.

    python3 make_screw_and_gear.py

writes the model and the meshes next to this file. The axes are those of
Shift: z points down. The gears turn about the y axis (a gear's axis is the
z axis of its frame), and the screws advance along the x axis.
"""

import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# the helpers that write meshes are shared with the other generators
import sys
sys.path.insert(0, os.path.join(HERE, ".."))
import mesh_builder  # noqa: E402

# ----------------------------------------------------------------- gears

MODULE = 0.02            # tooth size: pitch diameter = module * teeth
PRESSURE_ANGLE = math.radians(20.0)
GEAR_THICKNESS = 0.04
BACKLASH = 0.0004        # tooth thinned by this much at the pitch circle
# the teeth and the colors of the gears of the train
GEARS = [("gear_a", 16, 1, (0.10, 0.25, 0.70)),
         ("gear_b", 24, 0, (0.75, 0.30, 0.04)),
         ("gear_c", 12, 1, (0.08, 0.50, 0.15)),
         ("gear_d", 36, 0, (0.55, 0.55, 0.58))]
# s = 1: a tooth is centered on the line of centers toward the next gear, the
# x axis of the gear; s = 0: a gap is (all the teeth counts are even, so the
# same is true of the opposite direction, toward the previous gear). A tooth
# faces a gap where two gears mesh, so s alternates along the train.


def write_gear(name, teeth, tooth_at_zero, color):
    mesh = mesh_builder.Mesh(name, HERE)
    mesh.material("steel", color, metal=0.6, rough=0.4)
    mesh.material("mark", (0.85, 0.05, 0.03), metal=0.0, rough=0.5)
    half = GEAR_THICKNESS / 2.0
    mesh_builder.prism(mesh, "steel", mesh_builder.gear_outline(teeth, MODULE, tooth_at_zero, PRESSURE_ANGLE, BACKLASH), -half, half)
    # a red mark on the face, along the x axis of the gear, to see it turn
    pitch_r = MODULE * teeth / 2.0
    mesh_builder.cylinder(mesh, "mark", (0.55 * pitch_r, 0.0, half + 0.003), (0, 0, 1),
             0.2 * pitch_r, 0.006)
    mesh.write()


# ---------------------------------------------------------------- screws

LEAD = 0.08              # the advance of the thread per turn
ROOT_R, CREST_R = 0.034, 0.05
ROWS_PER_LEAD = 20
AROUND = 48


def thread_radius(row):
    u = (row % ROWS_PER_LEAD) / ROWS_PER_LEAD
    ramp = 0.15
    if u < 0.25:
        return ROOT_R
    if u < 0.25 + ramp:
        return ROOT_R + (CREST_R - ROOT_R) * (u - 0.25) / ramp
    if u < 0.6:
        return CREST_R
    if u < 0.6 + ramp:
        return CREST_R - (CREST_R - ROOT_R) * (u - 0.6) / ramp
    return ROOT_R


def write_rod(name, hand, z_low, z_high, handle_at_low):
    """a threaded rod along z from z_low to z_high; hand is +1 for a
    right-handed thread (the helix advances along +z as it turns by the right
    hand rule about +z), -1 for a left-handed one"""
    mesh = mesh_builder.Mesh(name, HERE)
    mesh.material("steel", (0.62, 0.62, 0.64), metal=1.0, rough=0.35)
    mesh.material("handle", (0.75, 0.08, 0.05), metal=0.0, rough=0.5)
    dz = LEAD / ROWS_PER_LEAD
    shift = hand * ROWS_PER_LEAD

    def vertex(k, m):
        """position of the vertex of column k, row m; the column AROUND is
        the column 0 of the row m + shift"""
        if k >= AROUND:
            k, m = k - AROUND, m + shift
        t = 2 * math.pi * k / AROUND
        z = min(max((m + hand * ROWS_PER_LEAD * k / AROUND) * dz, z_low),
                z_high)
        r = thread_radius(m)
        return (r * math.cos(t), r * math.sin(t), z), (k, m)

    first = int(math.floor(z_low / dz)) - ROWS_PER_LEAD - 2
    last = int(math.ceil(z_high / dz)) + ROWS_PER_LEAD + 2
    sums = {}
    triangles = []
    for m in range(first, last):
        for k in range(AROUND):
            corners = [vertex(k, m), vertex(k + 1, m), vertex(k + 1, m + 1),
                       vertex(k, m + 1)]
            for tri in ((0, 1, 2), (0, 2, 3)):
                p = [corners[i][0] for i in tri]
                key = [corners[i][1] for i in tri]
                u = [p[1][i] - p[0][i] for i in range(3)]
                w = [p[2][i] - p[0][i] for i in range(3)]
                n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2],
                     u[0] * w[1] - u[1] * w[0]]
                # outward: away from the axis
                mid = [sum(q[i] for q in p) / 3 for i in range(2)]
                if n[0] * mid[0] + n[1] * mid[1] < 0:
                    n = [-x for x in n]
                triangles.append((p, key, n))
                for kk in key:
                    sums.setdefault(kk, [0.0, 0.0, 0.0])
                    for i in range(3):
                        sums[kk][i] += n[i]
    for p, key, n in triangles:
        mesh.triangle("steel", p, [sums[kk] for kk in key])
    # the ends of the rod
    for z, direction in ((z_low - 0.0003, -1), (z_high + 0.0003, 1)):
        for k in range(AROUND):
            a = (CREST_R * math.cos(2 * math.pi * k / AROUND),
                 CREST_R * math.sin(2 * math.pi * k / AROUND), z)
            b = (CREST_R * math.cos(2 * math.pi * (k + 1) / AROUND),
                 CREST_R * math.sin(2 * math.pi * (k + 1) / AROUND), z)
            mesh.triangle("steel", [(0.0, 0.0, z), a, b],
                          [(0, 0, direction)] * 3)
    # the handle: a collar and a bar across the x axis with a ball on each end
    end = z_low if handle_at_low else z_high
    out = -1.0 if handle_at_low else 1.0
    mesh_builder.cylinder(mesh, "handle", (0.0, 0.0, end + out * 0.02), (0, 0, 1), 0.065,
             0.04)
    z_bar = end + out * 0.055
    mesh_builder.cylinder(mesh, "handle", (0.0, 0.0, z_bar), (1, 0, 0), 0.016, 0.5)
    for side in (-1, 1):
        mesh_builder.sphere(mesh, "handle", (side * 0.25, 0.0, z_bar), 0.032)
    mesh.write()


def write_nut():
    mesh = mesh_builder.Mesh("nut", HERE)
    mesh.material("brass", (0.80, 0.55, 0.14), metal=1.0, rough=0.4)
    apothem = 0.09
    corner = apothem / math.cos(math.pi / 6)
    outline = []
    # a hexagon, sampled at the angles of 48 points so it can be ringed
    # with the bore
    for k in range(48):
        t = 2 * math.pi * k / 48
        d = (t + math.pi / 6) % (math.pi / 3) - math.pi / 6
        r = apothem / math.cos(d)
        outline.append((r * math.cos(t), r * math.sin(t)))
    mesh_builder.prism(mesh, "brass", outline, -0.05, 0.05, hole=ROOT_R)
    mesh.write()
    return corner


# ------------------------------------------------------------- the model

R_GEAR = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]   # frame: x = x, y = z, z = -y
R_SCREW = [[0, 0, 1], [1, 0, 0], [0, 1, 0]]   # frame: x = y, y = z, z = x


def disc(mass, radius, thickness):
    return {"mass": mass,
            "Ixx": mass * (3 * radius ** 2 + thickness ** 2) / 12,
            "Iyy": mass * (3 * radius ** 2 + thickness ** 2) / 12,
            "Izz": 0.5 * mass * radius ** 2,
            "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0}


def weld(name, body, frame):
    return {"name": f"{name}_fixed", "type": "rigid",
            "parent": [["fixed"], frame], "child": [[body], "origin"]}


frames = [{"name": "overhead_1", "T": [6.0, -6.0, -6.0]},
          {"name": "overhead_2", "T": [-6.0, 6.0, -6.0]}]
bodies = []
joints = []

# ---- the gear train: gears side by side along x, their axes along y
SPEED = 1.5                      # rad/s of the first gear
x = 0.0
centers = []
radii = []
for name, teeth, tooth_at_zero, color in GEARS:
    radius = MODULE * teeth / 2.0
    if centers:
        x += radii[-1] + radius
    centers.append(x)
    radii.append(radius)
    write_gear(name, teeth, tooth_at_zero, color)

rates = []        # angular velocity of each gear
omega = SPEED
for i, (name, teeth, _, _) in enumerate(GEARS):
    if i > 0:
        omega *= -GEARS[i - 1][1] / teeth     # external pairs reverse
    rates.append(omega)
    frames.append({"name": f"{name}_axis", "T": [centers[i], 0.0, 0.0],
                   "R": R_GEAR})
    bodies.append({
        "name": name,
        "geometry": [{"type": "mesh", "filename": [f"{name}.obj", f"{name}.mtl"]}],
        "center_of_mass": [0.0, 0.0, 0.0],
        "inertia": disc(7850.0 * math.pi * radii[i] ** 2 * GEAR_THICKNESS,
                        radii[i], GEAR_THICKNESS)})
    joints.append({"name": f"{name}_bearing", "type": "revolute",
                   "axis": [0, 0, 1], "initial_displacement": 0.0,
                   "initial_velocity": rates[i],
                   "parent": [["fixed"], f"{name}_axis"],
                   "child": [[name], "origin"]})
for i in range(1, len(GEARS)):
    parent, child = GEARS[i - 1], GEARS[i]
    ratio = parent[1] / child[1]
    joints.append({
        "name": f"{parent[0]}_to_{child[0]}", "type": "gear",
        "separation": radii[i - 1] + radii[i], "gear_ratio": ratio,
        "initial_displacement": 0.0,
        # the line of centers turns at minus the rate of the parent gear
        "initial_velocity": -rates[i - 1],
        "parent": [[parent[0]], "origin"], "child": [[child[0]], "origin"]})

# a board behind the gears, for show
board_x = 0.5 * (centers[0] - radii[0] + centers[-1] + radii[-1])
board_w = centers[-1] - centers[0] + radii[0] + radii[-1] + 0.3
board_h = 2 * max(radii) + 0.3
frames.append({"name": "board", "T": [board_x, GEAR_THICKNESS / 2.0 + 0.013, 0.0],
               "R": R_GEAR})
bodies.append({
    "name": "board",
    "geometry": [{"type": "mesh",
                  "filename": ["../force_modules/meshes/box_black.obj"],
                  "scale": [board_w, board_h, 0.02],
                  "texture_mapping": "triplanar", "texture_scale": 0.4}],
    "center_of_mass": [0.0, 0.0, 0.0],
    "inertia": disc(10.0, 1.0, 0.02)})
joints.append(weld("board", "board", "board"))

# ---- the screws: a right-handed one and a left-handed one, below the gears
NUT_X = 0.7
SPIN = 5.0                       # rad/s
PITCH = LEAD / (2.0 * math.pi)   # advance per radian
write_nut()
screws = [("right", 1, 0.85, -1.0, 0.2, True),
          ("left", -1, 1.30, -0.2, 1.0, False)]
for side, hand, z, z_low, z_high, handle_at_low in screws:
    write_rod(f"rod_{side}", hand, z_low, z_high, handle_at_low)
    frames.append({"name": f"screw_{side}", "T": [NUT_X, 0.0, z],
                   "R": R_SCREW})
    bodies.append({
        "name": f"nut_{side}",
        "geometry": [{"type": "mesh", "filename": ["nut.obj", "nut.mtl"]}],
        "center_of_mass": [0.0, 0.0, 0.0],
        "inertia": disc(2.0, 0.09, 0.1)})
    joints.append(weld(f"nut_{side}", f"nut_{side}", f"screw_{side}"))
    mid = 0.5 * (z_low + z_high)
    handle = z_low if handle_at_low else z_high
    rod_mass, handle_mass = 47.0, 3.0
    com = (rod_mass * mid + handle_mass * handle) / (rod_mass + handle_mass)
    bodies.append({
        "name": f"rod_{side}",
        "geometry": [{"type": "mesh",
                      "filename": [f"rod_{side}.obj", f"rod_{side}.mtl"]}],
        "center_of_mass": [0.0, 0.0, com],
        "inertia": {"mass": 50.0, "Ixx": 6.7, "Iyy": 6.7, "Izz": 0.10,
                    "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0}})
    joints.append({"name": f"screw_{side}", "type": "screw", "axis": [0, 0, 1],
                   "pitch": hand * PITCH, "initial_displacement": 0.0,
                   "initial_velocity": SPIN,
                   "parent": [["fixed"], f"screw_{side}"],
                   "child": [[f"rod_{side}"], "origin"]})
# the bracket the nuts are held in
frames.append({"name": "bracket", "T": [NUT_X + 0.075, 0.0, 1.075],
               "R": R_SCREW})
bodies.append({
    "name": "bracket",
    "geometry": [{"type": "mesh",
                  "filename": ["../force_modules/meshes/box_black.obj"],
                  "scale": [0.5, 0.9, 0.03],
                  "texture_mapping": "triplanar", "texture_scale": 0.4}],
    "center_of_mass": [0.0, 0.0, 0.0],
    "inertia": disc(10.0, 0.5, 0.03)})
joints.append(weld("bracket", "bracket", "bracket"))

source = json.loads("".join(
    line for line in open(os.path.join(HERE, "../klann_linkage/klann_linkage.json"))
    if not line.strip().startswith("//")))

model = {
    "model_parameters": {
        "version": "0.2",
        "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                  "time": "seconds"},
        # z points down: gravity acts along +z
        "gravity": [0, 0, 9.81],
        "include_directories": ["."],
        "solvers": [{"type": "rnca"}],
        "integrators": [{"type": "ode113", "time_span": 8.0, "time_step": 0.02,
                         "relative_error": 1e-08, "absolute_error": 1e-08}],
    },
    "frames": frames,
    "lights": source["lights"],
    "assemblies": [{"name": "screw_and_gear", "bodies": bodies,
                    "joints": joints}],
}

header = """\
// The screw and gear joints, for inspection (written by
// make_screw_and_gear.py).
//
// Gear train: four spur gears on fixed axes, of 16, 24, 12 and 36 teeth. Each
// turns on a revolute joint, and a gear joint between each pair in mesh closes
// a loop: it makes the child gear roll on the parent without slipping, so
// the teeth of the meshes mesh at every instant, and the train has one degree
// of freedom. Gear a is given a speed; b turns the other way at 2/3 of its
// speed, c at 4/3, d at 4/9. The red marks turn with the gears.
//
// Screws: a rod with a right-handed thread and one with a left-handed thread,
// each on a screw joint to a nut in a bracket, with a handle. Both turn at
// 5 rad/s about the x axis. The pitch of a screw joint is the advance per
// radian: 0.08 m per turn / 2 pi. The right-handed rod advances along +x
// (toward the handle's side), the left-handed rod, with the pitch of opposite
// sign, along -x.
"""
with open(os.path.join(HERE, "screw_and_gear.json"), "w") as f:
    f.write(header)
    json.dump(model, f, indent=2)
    f.write("\n")
