#!/usr/bin/env python3
"""Write klann_walker.json: a walker of eight Klann legs, four on each side.

The legs are those of klann_linkage.json (read from it: its bodies, meshes
and joints), hung from a chassis on a float joint:

* two shafts, one at the front and one at the rear of the chassis, each
  carrying four legs, two on each side in their own planes, with the crank of
  every leg fixed to its shaft at its own phase (0 and 180 degrees on each
  side); the cranks of the rear shaft point the same way as those of the front
  one, and stay so, as the shafts turn alike;
* the legs of the right side are mirror images across the x-y plane of the
  linkage: meshes with scale [1, 1, -1], frames and centers of mass with z
  negated, and the products of inertia with z negated;
* the model follows shift's axes (x forward, y to the right, z down), and the
  linkage is turned into them: it is planar in its x-y plane with its y axis
  pointing down and the leg above the crank, so its x axis points to the back
  of the walker, its y axis up and its z axis (the thickness) outward on the
  left side, which hangs the leg below the crank;
* five gears of the same size on each side, outside the plates, as thick as
  a link: in a row, the gear of the rear shaft, an idler, the gear of a
  center shaft, an idler and the gear of the front shaft. The idlers are on
  pins through the plates. Neighboring gears are joined by gear joints (each
  closes a loop, and the teeth mesh at every instant). A single PID
  controller drives the center shaft to a speed, from a speed sensor, and
  the front and rear shafts turn through the idlers at the same speed and
  in the same direction;
* only the tip of each leg takes part in contact: a cylinder, with its axis
  across the leg, against the ground plane. The contact force module is
  Hertz line contact of the cylinder on the plane (see the Contact page).

The joint angles of the phases are from klann_linkage.json run with its crank
driven at a steady speed: the closure of each loop starts from them. PHASES,
of the front legs, are at the crank angles 0 and 180 degrees after its initial
one. A rear leg is a mirror image, so its crank points at pi - a for the angle
a of the linkage: REAR_PHASES, for the same direction of the crank as the
front leg of that phase, are at pi - a.

usage: make_klann_walker.py
"""

import copy
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# the helpers that write meshes are shared with the other generators
sys.path.insert(0, os.path.join(HERE, ".."))
import mesh_builder  # noqa: E402

SHAFT_X = {"f": 0.7, "b": -0.7}     # of the front and rear (back) shafts
LEG_Y = [0.15, 0.65]                # of the left legs (to the left), mirrored
PHASE = [0, 180]                    # crank angle of each leg on its shaft
PIVOT_Z = 0.9                       # of the shaft, below the deck
LINK_MASS_SCALE = 1000.0            # the single leg has the mass of a toy
KP = 3000.0                         # of the one PID, on the center shaft
KI = 6000.0
CRANK_SPEED = -3.0                  # of the three shafts, which turn alike
GEAR_TEETH = 20                     # of the five gears on each side
GEAR_MODULE = 0.0175                # pitch diameter = module * teeth = 0.35 m
GEAR_SHEET_MASS = 16.0              # kg per square meter of gear
GROUND_Z = 1.9                      # the ground surface, z points down
CHASSIS_Z = 0.2                     # initial z of the chassis origin
FOOT_RADIUS = 0.05
FOOT_CENTER = [-1.63, -0.42, 0.05]  # the center of the tip of the leg

# crank, second rocker arm, first rocker arm, crank to connecting rod, second
# rocker arm to connecting rod, first rocker arm to leg, connecting rod to leg
PHASES = {
    0: [3.532, 2.3624, 2.4979, 2.4919, 3.6615, 4.5717, 1.0458],
    180: [6.6744, 0.6592, 2.4779, -0.2521, 5.7631, 5.1328, 1.1884],
}
REAR_PHASES = {
    0: [5.8928, 0.6298, 2.093, 0.1945, 5.4575, 5.4003, 1.406],
    180: [9.0344, 2.4343, 2.9034, -2.6329, 3.9672, 4.1919, 0.6938],
}
JOINTS = [
    "a_fixed_to_crank", "b_fixed_to_second_rocker_arm",
    "c_fixed_to_first_rocker_arm", "d_crank_to_connecting_rod",
    "e_second_rocker_arm_to_connecting_rod", "f_first_rocker_arm_to_leg",
    "g_connecting_rod_to_leg"]

# the axes of the linkage in those of the chassis, by columns: x to -x,
# y to -z, z to -y
TURN = [[-1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, -1.0, 0.0]]

GEAR_THICKNESS = 0.1                # as thick as a link
GEAR_PITCH_R = GEAR_MODULE * GEAR_TEETH / 2.0
Z_HALF = GEAR_THICKNESS / 2.0
GEAR_Y = LEG_Y[0] + 0.3 + 0.1 + GEAR_THICKNESS / 2   # beside the plate
RZ_PI = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
# five gears of equal size in a row on each side, in mesh: the rear, an idler,
# the center, an idler and the front gear, a pitch diameter apart
assert abs(4 * GEAR_PITCH_R - SHAFT_X["f"]) < 1e-9
assert abs(4 * GEAR_PITCH_R + SHAFT_X["b"]) < 1e-9
IDLER_X = {"f": SHAFT_X["f"] / 2.0, "b": SHAFT_X["b"] / 2.0}


def write_gear(name, tooth_at_zero, color, pin=False):
    """a gear of the shafts, centered on its origin with its axis along z.
    One of three gears in a row has a tooth on the line of centers and the
    next ones a gap; their teeth then mesh (an even number of teeth puts a
    tooth, or a gap, on each side)"""
    mesh = mesh_builder.Mesh(name, HERE)
    mesh.material("steel", color, metal=0.6, rough=0.4)
    mesh_builder.prism(mesh, "steel",
                       mesh_builder.gear_outline(GEAR_TEETH, GEAR_MODULE,
                                                 tooth_at_zero,
                                                 backlash=0.0004),
                       -Z_HALF, Z_HALF)
    if pin:
        # an idler turns on a pin through the plate: a short bar through it
        mesh.material("pin", (0.62, 0.62, 0.64), metal=1.0, rough=0.35)
        mesh_builder.cylinder(mesh, "pin", (0.0, 0.0, 0.0), (0, 0, 1), 0.02,
                              0.24)
    mesh.write()


# the gears on the shafts have a tooth on the line of centers, the idlers
# between them a gap
write_gear("walker_gear_drive", True, (0.10, 0.25, 0.70))
write_gear("walker_gear_idle", False, (0.75, 0.30, 0.04), pin=True)


def gear_body(name, kind, frames):
    area = math.pi * GEAR_PITCH_R ** 2
    mass = GEAR_SHEET_MASS * area
    return {
        "name": name,
        "geometry": [{"type": "mesh",
                      "filename": [f"walker_gear_{kind}.obj",
                                   f"walker_gear_{kind}.mtl"]}],
        "frames": frames,
        "center_of_mass": [0.0, 0.0, 0.0],
        "inertia": {"mass": mass,
                    "Ixx": mass * (3 * GEAR_PITCH_R ** 2 + GEAR_THICKNESS ** 2) / 12,
                    "Iyy": mass * (3 * GEAR_PITCH_R ** 2 + GEAR_THICKNESS ** 2) / 12,
                    "Izz": 0.5 * mass * GEAR_PITCH_R ** 2,
                    "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0}}


# a shaft carries a gear on each side, welded to it; the gears are in the
# planes y = -GEAR_Y (left, z = +GEAR_Y on the shaft) and y = +GEAR_Y
GEAR_SIDES = (("l", GEAR_Y), ("r", -GEAR_Y))


def add_gears(shaft_id, kind, shaft_frames, mesh_frames):
    """the gears of the shaft on both sides, and their welds to the shaft;
    mesh_frames are the frames of a gear for the gear joints"""
    for side, z in GEAR_SIDES:
        name = f"{shaft_id}{side}_gear"
        shaft_frames.append({"name": name, "T": [0.0, 0.0, z]})
        bodies.append(gear_body(name, kind, copy.deepcopy(mesh_frames)))
        joints.append({"name": f"{name}_fixed", "type": "rigid",
                       "parent": [[f"{shaft_id}_shaft"], name],
                       "child": [[name], "origin"]})


source = json.loads("".join(
    line for line in open(os.path.join(HERE, "klann_linkage.json"))
    if not line.strip().startswith("//")))
assembly = source["assemblies"][0]
bodies_of = {b["name"]: b for b in assembly["bodies"]}
joints_of = {j["name"]: j for j in assembly["joints"]}


def mirror_body(body, across_xy, across_yz):
    """the mirror image of a body across its x-y plane (the right hand legs
    from the left hand ones) and/or its y-z plane (the rear legs from the
    front ones)"""
    body = copy.deepcopy(body)
    sx = -1 if across_yz else 1
    sz = -1 if across_xy else 1
    for geometry in body["geometry"]:
        geometry["scale"] = [sx, 1, sz]
    for frame in body.get("frames", []):
        frame["T"][0] *= sx
        frame["T"][2] *= sz
    body["center_of_mass"][0] *= sx
    body["center_of_mass"][2] *= sz
    inertia = body["inertia"]
    inertia["Ixy"] *= sx
    inertia["Iyz"] *= sz
    inertia["Izx"] *= sx * sz
    return body


def leg_bodies(prefix, mirrored, back, swap):
    bodies = []
    for name, body in bodies_of.items():
        body = copy.deepcopy(body)
        if swap:
            # the pins of the swapped layers are 0.1 m to the other side
            for frame in body.get("frames", []):
                if frame["name"] in ("crank_distal", "second_rocker_arm_distal",
                                     "first_rocker_arm_distal"):
                    frame["T"][2] = -0.1
                if frame["name"] == "connecting_rod_distal_2":
                    frame["T"][2] = -0.1
        body = mirror_body(body, mirrored, back)
        body["name"] = f"{prefix}_{name}"
        for frame in body.get("frames", []):
            frame["name"] = f"{frame['name']}"
        inertia = body["inertia"]
        inertia["mass"] *= LINK_MASS_SCALE
        for key in ("Ixx", "Iyy", "Izz", "Ixy", "Iyz", "Izx"):
            inertia[key] *= LINK_MASS_SCALE
        if name == "crank":
            # the crank of the single leg has no inertia: a bar 0.27 m long
            inertia.update({"mass": 0.12, "Ixx": 0.0016, "Iyy": 0.0016,
                            "Izz": 0.003, "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0})
        if name == "leg":
            sign = -1 if mirrored else 1
            body["collision_geometry"] = [{
                "name": f"{prefix}_foot", "type": "cylinder",
                "radius": FOOT_RADIUS, "half_length": 0.05,
                "pose": {"T": [(-1 if back else 1) * FOOT_CENTER[0], FOOT_CENTER[1],
                               sign * FOOT_CENTER[2]]},
                "material": "foot"}]
        bodies.append(body)
    return bodies


chassis_frames = []
bodies = []
joints = []
sensors = []
controllers = []
fx_of = {"f": 1.0, "b": -1.0}
for shaft_id, x0 in SHAFT_X.items():
    shaft_frames = []
    chassis_frames.append({"name": f"{shaft_id}_shaft", "T": [x0, 0.0, PIVOT_Z],
                           "R": TURN})
    for side, mirrored in (("l", False), ("r", True)):
        for k in range(2):
            prefix = f"{shaft_id}{side}{k + 1}"
            back = shaft_id == "b"
            # the rear legs are the mirror images of the front ones across
            # the y-z plane: x and the angles reverse
            fx = -1.0 if back else 1.0
            angles = [fx * a for a in
                      (REAR_PHASES if back else PHASES)[PHASE[k]]]
            y = LEG_Y[k] if mirrored else -LEG_Y[k]
            out = 1.0 if mirrored else -1.0   # outward along y
            # the layers of the inner legs are mirrored about the plane of
            # the connecting rod (the blue link, which stays): the crank and
            # the second rocker arm, 0.2 m outward, and the leg, 0.2 m
            # inward, change places
            swap = k == 0
            layer = {"crank": 0.2, "second_rocker": 0.2} if swap else {}
            # the fixed pivots of the rocker arms on the chassis, the frames
            # of the linkage turned; the crank is on the shaft
            frames = {
                "second_rocker": [x0 + fx * 0.59,
                                  y + layer.get("second_rocker", 0.0) * out,
                                  PIVOT_Z + 0.176],
                "first_rocker": [x0 + fx * 0.233, y + 0.1 * out, PIVOT_Z - 0.616],
            }
            for key, t in frames.items():
                chassis_frames.append({"name": f"{prefix}_{key}", "T": t,
                                       "R": TURN})
            # the crank frame on the shaft (its z axis is the shaft, outward
            # is -y of the chassis, so +z on the left): turned by the crank
            # angle
            c, sn = math.cos(angles[0]), math.sin(angles[0])
            shaft_frames.append({
                "name": f"{prefix}_crank",
                "T": [0.0, 0.0, -(y + layer.get("crank", 0.0) * out)],
                "R": [[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]]})
            bodies += leg_bodies(prefix, mirrored, back, swap)
            for name, angle in zip(JOINTS, angles):
                joint = copy.deepcopy(joints_of[name])
                joint["name"] = f"{prefix}_{name}"
                for end in ("parent", "child"):
                    (body,), frame = joint[end]
                    if body == "fixed":
                        key = {"origin": "crank",
                               "second_rocker_arm_attachment": "second_rocker",
                               "first_rocker_arm_attachment":
                               "first_rocker"}[frame]
                        joint[end] = [
                            [f"{shaft_id}_shaft" if key == "crank"
                             else "chassis"], f"{prefix}_{key}"]
                    else:
                        joint[end] = [[f"{prefix}_{body}"], frame]
                if name == "a_fixed_to_crank":
                    # the crank is fixed to the shaft, at its angle
                    joint = {"name": joint["name"], "type": "rigid",
                             "parent": joint["parent"],
                             "child": joint["child"]}
                else:
                    joint["initial_displacement"] = angle
                    joint["initial_velocity"] = 0.0
                joints.append(joint)
    bodies.append({
        "name": f"{shaft_id}_shaft",
        # a bar along its z axis, the axis of the cranks
        "geometry": [{"type": "mesh",
                      "filename":
                      ["../force_modules/meshes/cylinder_z_aluminium.obj"],
                      "scale": [0.03, 0.03, 2 * GEAR_Y + GEAR_THICKNESS]}],
        "frames": shaft_frames,
        "center_of_mass": [0.0, 0.0, 0.0],
        "inertia": {"mass": 5.0, "Ixx": 1.5, "Iyy": 1.5, "Izz": 0.002,
                    "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0},
    })
    joints.append({"name": f"{shaft_id}_shaft_bearing", "type": "revolute",
                   "parent": [["chassis"], f"{shaft_id}_shaft"],
                   "child": [[f"{shaft_id}_shaft"], "origin"],
                   "axis": [0, 0, 1], "initial_displacement": 0.0,
                   "initial_velocity": 0.0})
    # a bar through the top pivots of the legs, for show: the legs turn on
    # it (their revolute joints are to the chassis frames at the pivots), it
    # drives nothing and is fixed to the chassis
    top_x = x0 + fx_of[shaft_id] * 0.233
    chassis_frames.append({"name": f"{shaft_id}_top_bar",
                           "T": [top_x, 0.0, PIVOT_Z - 0.616], "R": TURN})
    bodies.append({
        "name": f"{shaft_id}_top_bar",
        "geometry": [{"type": "mesh",
                      "filename":
                      ["../force_modules/meshes/cylinder_z_aluminium.obj"],
                      "scale": [0.03, 0.03, 1.7]}],
        "center_of_mass": [0.0, 0.0, 0.0],
        "inertia": {"mass": 5.0, "Ixx": 1.5, "Iyy": 1.5, "Izz": 0.002,
                    "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0},
    })
    joints.append({"name": f"{shaft_id}_top_bar_fixed", "type": "rigid",
                   "parent": [["chassis"], f"{shaft_id}_top_bar"],
                   "child": [[f"{shaft_id}_top_bar"], "origin"]})
    # the gears of the shaft, driven through an idler from the center
    # shaft: the frame "mesh" of a gear is where its joint to the idler is
    # (x toward the idler; the frames of all the gears are turned alike,
    # and x of them is toward the rear)
    add_gears(shaft_id, "drive", shaft_frames,
              [{"name": "mesh", "T": [0.0, 0.0, 0.0],
                "R": RZ_PI if shaft_id == "f" else [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}])

# the center shaft: the only one with a drive, a PID controller. Its gears
# drive those of the front and rear shafts through idlers, so all three
# shafts turn alike.
chassis_frames.append({"name": "c_shaft", "T": [0.0, 0.0, PIVOT_Z],
                       "R": TURN})
center_frames = []
bodies.append({
    "name": "c_shaft",
    "geometry": [{"type": "mesh",
                  "filename":
                  ["../force_modules/meshes/cylinder_z_aluminium.obj"],
                  "scale": [0.03, 0.03, 2 * GEAR_Y + GEAR_THICKNESS]}],
    "frames": center_frames,
    "center_of_mass": [0.0, 0.0, 0.0],
    "inertia": {"mass": 5.0, "Ixx": 1.5, "Iyy": 1.5, "Izz": 0.002,
                "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0},
})
joints.append({"name": "c_shaft_bearing", "type": "revolute",
               "parent": [["chassis"], "c_shaft"],
               "child": [["c_shaft"], "origin"],
               "axis": [0, 0, 1], "initial_displacement": 0.0,
               "initial_velocity": 0.0})
add_gears("c", "drive", center_frames,
          [{"name": "mesh_front", "T": [0.0, 0.0, 0.0], "R": RZ_PI},
           {"name": "mesh_rear", "T": [0.0, 0.0, 0.0]}])
# the idlers: two gears on each side, each on a pin through the plate, a
# revolute joint to the chassis; they mesh with the center gear and with the
# gear of the front or rear shaft
for side, z in GEAR_SIDES:
    for other, mesh_frame in (("f", "mesh_front"), ("b", "mesh_rear")):
        idler = f"i{other}{side}_gear"
        turned = RZ_PI if other == "f" else [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        chassis_frames.append({"name": idler,
                               "T": [IDLER_X[other], -z, PIVOT_Z],
                               "R": TURN})
        bodies.insert(1, gear_body(idler, "idle", [
            {"name": "mesh_in", "T": [0.0, 0.0, 0.0], "R": turned},
            {"name": "mesh_out", "T": [0.0, 0.0, 0.0], "R": turned}]))
        joints.append({"name": f"{idler}_bearing", "type": "revolute",
                       "parent": [["chassis"], idler],
                       "child": [[idler], "origin"],
                       "axis": [0, 0, 1], "initial_displacement": 0.0,
                       "initial_velocity": 0.0})
        # center gear to idler to the gear of the shaft; the pitch circles
        # of all the gears touch
        for parent, parent_frame, child, child_frame in (
                (f"c{side}_gear", mesh_frame, idler, "mesh_in"),
                (idler, "mesh_out", f"{other}{side}_gear", "mesh")):
            joints.append({
                "name": f"{parent}_to_{child}", "type": "gear",
                "separation": 2.0 * GEAR_PITCH_R, "gear_ratio": 1.0,
                "initial_displacement": 0.0, "initial_velocity": 0.0,
                "parent": [[parent], parent_frame],
                "child": [[child], child_frame]})
sensors.append({"type": "joint", "name": "c_shaft_speed",
                "joint": "c_shaft_bearing", "quantity": "velocity"})
controllers.append({"type": "pid", "name": "c_shaft_drive",
                    "kp": KP, "ki": KI, "kd": 0.0, "target": CRANK_SPEED,
                    "sensor": "c_shaft_speed", "joint": "c_shaft_bearing"})

# two plates, one on each side, standing beside the outer faces of the inner
# cranks: upside down trapezoids in the x-z plane of the chassis, as thick as
# a link, with their lower corners on the axes of the two crank shafts and
# their upper corners on those of the two top bars (the upper pivots)
PLATE_THICKNESS = 0.1
PLATE_SHEET_MASS = 16.0      # kg per square meter of the plate
PLATE_Y = LEG_Y[0] + 0.3 + PLATE_THICKNESS / 2    # the middle of the plate
lower_x = SHAFT_X["f"]
upper_x = SHAFT_X["f"] + fx_of["f"] * 0.233
lower_z = PIVOT_Z
upper_z = PIVOT_Z - 0.616


LINK_RADIUS = 0.05     # of the rounded ends of the links (half the width)
PIN_RADIUS = 0.025     # of the pin holes of the links


def triangulate(outline, holes):
    """triangles of a polygon (counter-clockwise points) with holes
    (clockwise point lists): holes are joined to the outline by bridges, and
    ears are clipped"""
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def crosses(p, q, a, b):
        if p in (a, b) or q in (a, b):
            return False
        return (cross(p, q, a) * cross(p, q, b) < 0 and
                cross(a, b, p) * cross(a, b, q) < 0)

    def inside(pt, polygon):
        count = False
        for k in range(len(polygon)):
            a, b = polygon[k], polygon[k - 1]
            if (a[1] > pt[1]) != (b[1] > pt[1]) and pt[0] < (
                    b[0] - a[0]) * (pt[1] - a[1]) / (b[1] - a[1]) + a[0]:
                count = not count
        return count

    polygon = list(outline)
    pending = [list(h) for h in holes]
    while pending:
        hole = pending.pop(0)
        edges = [(polygon[k - 1], polygon[k]) for k in range(len(polygon))]
        for other in pending:
            edges += [(other[k - 1], other[k]) for k in range(len(other))]
        edges += [(hole[k - 1], hole[k]) for k in range(len(hole))]
        best = None
        for hi, h in enumerate(hole):
            for vi, v in enumerate(polygon):
                d = (h[0] - v[0]) ** 2 + (h[1] - v[1]) ** 2
                if best is not None and d >= best[0]:
                    continue
                mid = ((h[0] + v[0]) / 2, (h[1] + v[1]) / 2)
                if (not inside(mid, polygon) or inside(mid, hole) or
                        any(inside(mid, o) for o in pending) or
                        any(crosses(h, v, a, b) for a, b in edges)):
                    continue
                best = (d, hi, vi)
        _, hi, vi = best
        polygon = (polygon[:vi + 1] + hole[hi:] + hole[:hi + 1] +
                   [polygon[vi]] + polygon[vi + 1:])
    triangles = []
    points = list(polygon)
    guard = 0
    while len(points) > 3:
        guard += 1
        if guard > 100000:
            raise RuntimeError("triangulation failed")
        n = len(points)
        for k in range(n):
            p, v, q = points[k - 1], points[k], points[(k + 1) % n]
            if cross(p, v, q) <= 1e-12:
                continue
            if any(x not in (p, v, q) and cross(p, v, x) >= -1e-12 and
                   cross(v, q, x) >= -1e-12 and cross(q, p, x) >= -1e-12
                   for x in points):
                continue
            triangles.append((p, v, q))
            del points[k]
            break
        else:
            # a degenerate sliver: drop the flattest point
            k = min(range(n), key=lambda m: abs(cross(
                points[m - 1], points[m], points[(m + 1) % n])))
            del points[k]
    triangles.append(tuple(points))
    return triangles


def plate_outline():
    """the corners of the plate on the pivots, and the pivots themselves, in
    (x, z) counter-clockwise as plotted with z up the page"""
    pivots = [(-lower_x, lower_z), (-upper_x, upper_z),
              (upper_x, upper_z), (lower_x, lower_z)]
    outline = []
    count = len(pivots)
    for k, (px, pz) in enumerate(pivots):
        a, b, c = pivots[k - 1], pivots[k], pivots[(k + 1) % count]

        def normal(p, q):
            dx, dz = q[0] - p[0], q[1] - p[1]
            length = math.hypot(dx, dz)
            return dz / length, -dx / length

        n0, n1 = normal(a, b), normal(b, c)
        t0 = math.atan2(n0[1], n0[0])
        t1 = math.atan2(n1[1], n1[0])
        while t1 < t0:
            t1 += 2.0 * math.pi
        steps = max(2, int((t1 - t0) / (math.pi / 2) * 12))
        for m in range(steps + 1):
            t = t0 + (t1 - t0) * m / steps
            outline.append((px + LINK_RADIUS * math.cos(t),
                            pz + LINK_RADIUS * math.sin(t)))
    return outline, pivots


def write_plate(path):
    """an OBJ of the plate: the trapezoid of the pivots grown by the radius
    of the links (rounded corners), with a pin hole at each pivot, centered
    in y"""
    h = PLATE_THICKNESS / 2
    outline, pivots = plate_outline()
    # the middle is cut out, leaving a frame of the width of a link whose
    # members run along the lines between the pivots: the edges of the
    # trapezoid moved inward by half the width, with sharp corners
    inner = []
    for k in range(len(pivots)):
        a, b, c = pivots[k - 1], pivots[k], pivots[(k + 1) % len(pivots)]
        lines_ = []
        for p, q in ((a, b), (b, c)):
            dx, dz = q[0] - p[0], q[1] - p[1]
            length = math.hypot(dx, dz)
            n = (dz / length, -dx / length)      # outward
            lines_.append(((p[0] - LINK_RADIUS * n[0],
                            p[1] - LINK_RADIUS * n[1]), (dx, dz)))
        (p0, d0), (p1, d1) = lines_
        det = d0[0] * d1[1] - d0[1] * d1[0]
        t = ((p1[0] - p0[0]) * d1[1] - (p1[1] - p0[1]) * d1[0]) / det
        inner.append((p0[0] + t * d0[0], p0[1] + t * d0[1]))
    holes = [inner[::-1]]                       # clockwise, a hole
    for px, pz in pivots + [(x, lower_z) for x in (-IDLER_X["f"], 0.0, IDLER_X["f"])]:   # and the center shaft and the idler pins
        holes.append([(px + PIN_RADIUS * math.cos(-2 * math.pi * m / 24),
                       pz + PIN_RADIUS * math.sin(-2 * math.pi * m / 24))
                      for m in range(24)])
    lines = ["# trapezoid plate of make_klann_walker.py",
             "mtllib trapezoid_plate.mtl", "o plate"]
    vertices, normals, faces = [], [], []

    def vertex(x, y, z):
        vertices.append((x, y, z))
        return len(vertices)

    def normal(nx, ny, nz):
        normals.append((nx, ny, nz))
        return len(normals)

    # the two faces, flat
    for sign in (-1.0, 1.0):
        n = normal(0.0, sign, 0.0)
        for tri in triangulate(outline, holes):
            tri = list(tri)
            # winding: the normal along +y for the face at y = +h; the
            # points are given in (x, z), where counter-clockwise is the
            # normal of -y in these axes
            if sign > 0:
                tri = tri[::-1]
            ids = [vertex(x, sign * h, z) for x, z in tri]
            faces.append([(i, n) for i in ids])
    # the walls, smooth around the arcs and the holes; outward from the
    # material, which is to the right of the outline and the left of the holes
    for index, loop in enumerate([outline] + holes):
        count = len(loop)
        ids = []
        flat = index == 1               # the cutout has sharp corners
        for k in range(count):
            a, b, c = loop[k - 1], loop[k], loop[(k + 1) % count]
            d0 = (b[0] - a[0], b[1] - a[1])
            d1 = (c[0] - b[0], c[1] - b[1])
            l0, l1 = math.hypot(*d0), math.hypot(*d1)
            if flat:                    # the normal of the edge from b to c
                nx, nz = d1[1] / l1, -d1[0] / l1
            else:
                nx = d0[1] / l0 + d1[1] / l1
                nz = -d0[0] / l0 - d1[0] / l1
            length = math.hypot(nx, nz)
            ids.append(normal(nx / length, 0.0, nz / length))
        for k in range(count):
            a, b = loop[k], loop[(k + 1) % count]
            after = ids[k] if flat else ids[(k + 1) % count]
            quad = [(vertex(a[0], -h, a[1]), ids[k]),
                    (vertex(b[0], -h, b[1]), after),
                    (vertex(b[0], h, b[1]), after),
                    (vertex(a[0], h, a[1]), ids[k])]
            # (x, y, z) from -y to +y: the wall normal is to the right of
            # the direction of travel in (x, z)
            faces.append([quad[0], quad[3], quad[2]])
            faces.append([quad[0], quad[2], quad[1]])
    lines += [f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in vertices]
    lines += [f"vn {x:.6f} {y:.6f} {z:.6f}" for x, y, z in normals]
    lines.append("usemtl None")
    for face in faces:
        lines.append("f " + " ".join(f"{v}//{n}" for v, n in face))
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    # area, centroid and second moments of the face, to state the mass
    # properties of the plate (a uniform sheet)
    area = sx = sz = sxx = szz = 0.0
    for tri in triangulate(outline, holes):
        (x1, z1), (x2, z2), (x3, z3) = tri
        a = abs((x2 - x1) * (z3 - z1) - (x3 - x1) * (z2 - z1)) / 2
        area += a
        sx += a * (x1 + x2 + x3) / 3
        sz += a * (z1 + z2 + z3) / 3
        sxx += a / 6 * (x1 * x1 + x2 * x2 + x3 * x3 +
                        x1 * x2 + x2 * x3 + x1 * x3)
        szz += a / 6 * (z1 * z1 + z2 * z2 + z3 * z3 +
                        z1 * z2 + z2 * z3 + z1 * z3)
    cz = sz / area
    return {"area": area, "cz": cz, "x2": sxx - sx * sx / area,
            "z2": szz - sz * sz / area}


plate = write_plate(os.path.join(HERE, "trapezoid_plate.obj"))
with open(os.path.join(HERE, "crank.mtl")) as f:
    open(os.path.join(HERE, "trapezoid_plate.mtl"), "w").write(f.read())

for side, sign in (("l", -1.0), ("r", 1.0)):
    chassis_frames.append({"name": f"{side}_plate",
                           "T": [0.0, sign * PLATE_Y, 0.0]})
    bodies.append({
        "name": f"{side}_plate",
        "geometry": [{"type": "mesh",
                      "filename": ["trapezoid_plate.obj",
                                   "trapezoid_plate.mtl"]}],
        # the frame as a uniform sheet, PLATE_SHEET_MASS kg per square meter
        "center_of_mass": [0.0, 0.0, plate["cz"]],
        "inertia": {"mass": PLATE_SHEET_MASS * plate["area"],
                    "Ixx": PLATE_SHEET_MASS * (
                        plate["z2"] + plate["area"] * PLATE_THICKNESS ** 2 / 12),
                    "Iyy": PLATE_SHEET_MASS * (plate["x2"] + plate["z2"]),
                    "Izz": PLATE_SHEET_MASS * (
                        plate["x2"] + plate["area"] * PLATE_THICKNESS ** 2 / 12),
                    "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0},
    })
    joints.append({"name": f"{side}_plate_fixed", "type": "rigid",
                   "parent": [["chassis"], f"{side}_plate"],
                   "child": [[f"{side}_plate"], "origin"]})

chassis = {
    "name": "chassis",
    "frames": chassis_frames,
    "center_of_mass": [0.0, 0.0, 0.4],
    "inertia": {"mass": 30.0, "Ixx": 29.0, "Iyy": 50.0, "Izz": 29.0,
                "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0},
}
bodies.insert(0, chassis)
joints.insert(0, {"name": "chassis_float", "type": "float",
                  "parent": [["fixed"], "origin"],
                  "child": [["chassis"], "origin"],
                  "initial_displacement": {"T": [0.0, 0.0, CHASSIS_Z]}})

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
        "integrators": [{"type": "ode113", "time_span": 10.0,
                         "time_step": 0.02, "relative_error": 1e-05,
                         "absolute_error": 1e-05}],
        "loggers": [{"type": "csv", "csv_logfile": "klann_walker.csv",
                     "timestamp_logfile": False}],
    },
    "frames": [{"name": "overhead_1", "T": [6.0, -6.0, -6.0]},
               {"name": "overhead_2", "T": [-6.0, 6.0, -6.0]}],
    "lights": source["lights"],
    # the ground, a plane with its normal up (-z) at z = GROUND_Z
    "collision_geometry": [{"name": "ground", "type": "plane",
                            "normal": [0, 0, -1], "offset": -GROUND_Z,
                            "material": "ground"}],
    "assemblies": [{
        "name": "klann_walker",
        "bodies": bodies,
        "joints": joints,
        "sensors": sensors,
        "controllers": controllers,
        "force_modules": [{
            "type": "contact", "name": "contact",
            # the foot of each leg on the ground: Hertz line contact of a
            # cylinder on a plane, with a damping a little higher than the
            # stiffness alone needs, so the integrator takes longer steps
            "damping_depth": 0.001,
            "materials": [
                {"name": "foot", "youngs_modulus": 2e6, "poisson_ratio": 0.45,
                 "damping": 12000.0, "friction": 0.9,
                 "contact_stiffness": 1e5},
                {"name": "ground", "youngs_modulus": 5e7,
                 "poisson_ratio": 0.3, "damping": 1e9, "friction": 0.9,
                 "contact_stiffness": 1e6},
            ],
        }],
    }],
}

header = """  // A walker of eight Klann legs (written by make_klann_walker.py from
  // klann_linkage.json): four legs on each side of a chassis that is free to
  // move, each leg in its own plane, the right legs mirror images of the left,
  // with the cranks of four legs fixed to each of two shafts, a front and a
  // rear one, which PID controllers drive. Only the tip of each leg, a cylinder, has contact, and
  // only with the ground plane."""
text = json.dumps(model, indent=2)
text = text.replace("{\n", "{\n" + header + "\n", 1)
with open(os.path.join(HERE, "klann_walker.json"), "w") as f:
    f.write(text + "\n")
