"""writes polo_r_wrc.json: the Volkswagen Polo R WRC on its own suspension
and drivetrain, over a gravel stage.

The source model has no suspension: only the body, the wheels and the brake
calipers. The suspension is added here, MacPherson struts front and rear as on
the car, with hardpoints laid out from the meshes (the wheels, the calipers
and the wheel arches) and the published dimensions; the parts it adds are not
drawn. The meshes come from shift/scripts/vehicle_splits/polo_r_wrc.json (see
readme.rst).

Published figures (Volkswagen Motorsport, 2014 FIA World Rally Championship
press kit, technical specifications), used as given:

- engine: 1.6 l turbocharged straight four, 33 mm restrictor, 232 kW at
  6,250 rpm, 425 Nm at 5,000 rpm, transversally mounted ahead of the front
  axle;
- transmission: six speed sequential; hydraulically actuated double disc
  sintered metal clutch (ZF); permanent four wheel drive with a fixed drive
  between the front and rear axles (no center differential), multi-plate
  limited slip differentials front and rear;
- suspension: McPherson struts front and rear, ZF dampers; travel about
  275 mm on gravel;
- brakes: ventilated discs, 300 mm front and rear on gravel; wheels 7 x 15
  inch on gravel;
- track 1,610 mm, wheelbase 2,480 mm, minimum weight 1,200 kg (without crew).

The gear ratios, final drive, masses and inertias, springs, dampers, tire
properties and the hardpoints are estimates (the ratios are not published).
The meshes have a wheelbase of 2.41 m and a track of 1.495 m at the middle of
the tires, a few percent less than the car; they are not rescaled.

Vehicle (SAE) coordinates: x forward, y right, z down, in meters, with the
origin on the ground between the axles. Hardpoints are given for the right
side; the left side is their mirror image in y.

Each corner: the strut has a ball joint (spherical) at its top mount on the
body and slides, sprung and damped, along its axis to the knuckle, to which it
is clamped. A lower arm turns on the body about an axis along x and carries
the knuckle on a ball joint. A tie rod, with a ball joint at its inner end and
a universal joint at the knuckle, steers the knuckle from the rack (front) or
holds its toe from the body (rear). This leaves each knuckle free to rise and
fall, and the front ones to steer.

The route is a list of schedules: the throttle, the clutch, the gear and the
brakes, and the steering, which sets the target of a PID controller that holds
the rack in position.

    python3 generate_polo_r_wrc.py polo_r_wrc.json

It also writes polo_r_wrc_suspension.json, the same car without its body
shell, and the meshes of the parts it adds (OBJ).
"""
import json
import math
import os
import struct
import sys

import numpy as np
from PIL import Image, ImageOps

HERE = os.path.dirname(os.path.abspath(__file__))
# the helpers that write meshes are shared with the other generators
sys.path.insert(0, os.path.join(HERE, ".."))
import mesh_builder  # noqa: E402

out = sys.argv[1] if len(sys.argv) > 1 else "polo_r_wrc.json"
radius = 0.314     # gravel tire on a 15 inch wheel, from the meshes
# the tires start touching the ground: the chassis starts a little lower,
# for the tires to carry the car
ride_drop = 0.02

# the drive, as schedules [t0, v0, t1, v1, ...]: the throttle (0 to 1), the
# clutch (0 to 1), the gear, the brakes (0 to 1) and the rack position (m).
# The tie rods are behind the front axle, so a positive rack position (to the
# right) turns the car left
ROUTE = {
    "time_span": 30.0,
    "throttle": [0, 0, 0.3, 0.6, 0.8, 1.0,
                 9.9, 1.0, 10.1, 0.0, 12.2, 0.0, 12.8, 0.35, 14.0, 0.5,
                 14.6, 1.0,
                 16.6, 1.0, 16.8, 0.4, 17.8, 0.4, 18.2, 1.0,
                 25.4, 1.0, 25.6, 0.0, 30, 0.0],
    "clutch": [0, 0, 0.3, 0, 1.0, 1.0, 27.4, 1.0, 27.7, 0, 30, 0],
    "gears": [0, 1, 2.6, 2, 4.2, 3, 6.4, 4, 8.8, 5, 10.5, 4, 11.0, 3,
              11.5, 2, 14.0, 3, 16.4, 4, 20.0, 5, 26.0, 4, 26.6, 3, 27.2, 2],
    "brakes": [0, 0, 10.0, 0, 10.2, 0.75, 11.8, 0.75, 12.1, 0,
               25.5, 0, 25.7, 0.8, 30, 0.8],
    # a fast left, a tight right after the braking, a left-right chicane and
    # a long left before the final braking
    "rack": [0, 0, 5.5, 0, 6.5, 0.012, 8.5, 0.012, 9.5, 0,
             11.2, 0, 12.0, -0.04, 14.2, -0.04, 15.0, 0,
             15.2, 0, 15.8, 0.02, 16.4, 0.02, 17.0, -0.016, 17.8, -0.016,
             18.4, 0, 21.0, 0, 22.0, 0.012, 24.4, 0.012, 25.2, 0, 30, 0],
}

# the stage: samples on a grid over the route, and the waves that make it,
# each an amplitude (m), a wavelength (m), the heading it runs along (degrees)
# and a phase (radians). The long waves are hills, crests and dips; the short
# ones, shorter than the wheelbase, are the bumps and ruts of a gravel road,
# which the Iowa tire follows under its contact patch. The crests are jumps
# across the route where the car is fast and straight, each a center x, y
# (m), the heading of the route over it (degrees), a height, a length along
# the route and a width (m): over the top of a crest the car would need more
# than g to follow the ground (curvature height (2 pi / length)^2 / 2 above
# g / v^2), so it leaves it, for about 0.6 s before the first braking and
# 0.7 s after the chicane. Their places follow the route: a change to the car
# or to the route can miss them.
TERRAIN = {
    "origin": [-40.0, -250.0],
    "spacing": 1.0,
    "size": [851, 501],
    "flat_radius": 10.0,
    "full_radius": 40.0,
    "waves": [[1.50, 190.0, 35.0, 1.1],
              [0.80, 80.0, 10.0, 0.4], [0.40, 47.0, 100.0, 1.7],
              [0.02, 7.0, 25.0, 0.9], [0.015, 5.2, -65.0, 2.3]],
    "crests": [[171.0, -24.0, -34.0, 0.35, 24.0, 30.0],
               [422.0, -58.0, 23.0, 0.45, 24.0, 30.0]],
}


def add(a, b):
    return [a[i] + b[i] for i in range(3)]


def sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def unit(a):
    n = math.sqrt(sum(x * x for x in a))
    return [x / n for x in a]


def norm(a):
    return math.sqrt(sum(x * x for x in a))


def r(v, n=6):
    return [round(x, n) + 0.0 for x in v]


def inertia(m, ixx, iyy, izz):
    return {"mass": m, "Ixx": ixx, "Iyy": iyy, "Izz": izz,
            "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0}


# right side hardpoints. The wheel and knuckle match the "bodies" of the
# split configuration. The lower ball joint sits inside the 15 inch rim; the
# strut top under the wing, giving about 10 degrees of kingpin inclination;
# the tie rod behind the axle, with its inner end near the height of the lower
# arm's pivot for little bump steer. Strut: spring rate and damping along it,
# and its force at the modelled length (the preload, which holds that length
# with the car at rest)
def corner(x, k, c, preload, tie_rod_x, bar_stiffness, bar_diameter):
    toward = -1.0 if x > 0.0 else 1.0  # towards the middle of the car
    return {
        "wheel": [x, 0.7475, -0.314],
        "knuckle": [x, 0.66, -0.314],
        "lower_arm": [x, 0.33, -0.19],          # inner pivot, along x
        "lower_ball_joint": [x + 0.01, 0.69, -0.17],
        "strut_top": [x - 0.025, 0.57, -0.80],
        "strut_bottom": [x - 0.005, 0.635, -0.42],  # clamp on the knuckle
        "tie_rod_outer": [tie_rod_x, 0.66, -0.26],
        "tie_rod_inner": [tie_rod_x, 0.30, -0.24],
        # anti-roll bar: its axis along y, inboard of the axle; its arm
        # runs out to the drop link, which stands on the lower arm
        "bar": [x + 0.225 * toward, 0.0, -0.30],
        "bar_end": [x + 0.225 * toward, 0.45, -0.30],
        "link_top": [x + 0.08 * toward, 0.50, -0.30],
        "link_bottom": [x + 0.08 * toward, 0.50, -0.18],
        "k": k, "c": c, "preload": preload, "bar_stiffness": bar_stiffness,
        "bar_radius": 0.5 * bar_diameter}


# about 1.6 Hz front and 1.7 Hz rear in heave, a third of critical damping:
# a gravel setup, soft and with long travel
# The anti-roll bars' torsional stiffness (N m/rad) is that of solid steel
# bars about 17.5 and 15.5 mm across and 0.9 m long; through the drop links
# it adds about half and a third to the roll stiffness of the springs
front = corner(1.205, 36000.0, 2600.0, 3458.0, 1.07, 800.0, 0.0175)
rear = corner(-1.205, 34000.0, 2300.0, 2942.0, -1.34, 500.0, 0.0155)
# the bump stops on the strut's rod: they touch the top of the damper tube
# 100 mm into the stroke from rest, and stiffen as they are squeezed
# (polyurethane: force against compression past the edge, m and N); the
# rebound stops in the dampers hold the strut 130 mm out from rest
BUMP_GAP, REBOUND_GAP = 0.10, 0.13
BUMP_CURVE = [-0.04, -60000.0, -0.03, -24000.0, -0.02, -8000.0,
              -0.01, -2000.0, 0.0, 0.0]
rack_origin = [1.07, 0.0, -0.24]
# the halfshafts run from the differentials, near the middle of the car, to
# the hubs; the prop shaft joins the differentials along the tunnel
halfshaft_inner_y, halfshaft_z = 0.16, -0.30

# masses (kg): with the crew and some fuel the car is about 1390 kg
masses = {"chassis": 1218.0, "wheel": 23.0, "knuckle": 12.0, "strut": 3.0,
          "lower_arm": 4.0, "tie_rod": 1.0, "bar": 1.2, "drop_link": 0.4}
chassis_com = [0.1, 0.0, -0.45]

bodies, joints, force_modules, controllers = [], [], [], []
chassis_frames, rack_frames = [], []
virtual = 0


def new_virtual():
    global virtual
    virtual += 1
    name = "v%d" % virtual
    bodies.append({"name": name})
    return name


def joint(name, kind, parent, child, **extra):
    j = {"name": name, "type": kind, "parent": parent, "child": child}
    j.update(extra)
    joints.append(j)


def mesh(name):
    return [{"type": "mesh", "filename": [name + ".glb"]}]


# the parts the source model lacks are drawn from cylinders, spheres and
# coils, as OBJ meshes in the coordinates of their bodies (the vehicle axes,
# from the body's origin)
parts = {}


def part(name):
    """the mesh of the added parts of a body, and its geometry entry"""
    if name not in parts:
        m = mesh_builder.Mesh(name, HERE)
        m.material("steel", (0.55, 0.56, 0.58), metal=0.8, rough=0.35)
        m.material("dark", (0.12, 0.12, 0.13), metal=0.3, rough=0.6)
        m.material("spring", (0.85, 0.12, 0.08), metal=0.2, rough=0.4)
        m.material("damper", (0.85, 0.65, 0.12), metal=0.6, rough=0.3)
        m.material("boot", (0.05, 0.05, 0.05), metal=0.0, rough=0.8)
        parts[name] = m
    return parts[name]


def drawn(name):
    return [{"type": "mesh", "filename": [name + ".obj"]}]


def bar(m, material, a, b, radius, sides=16):
    """a cylinder from a to b"""
    along = sub(b, a)
    mesh_builder.cylinder(m, material, [0.5 * (a[i] + b[i]) for i in range(3)],
                          unit(along), radius, norm(along), sides)


def ball(m, material, center, radius):
    mesh_builder.sphere(m, material, center, radius, rings=8, sides=14)


def coil(m, material, start, axis, coil_radius, wire, length, turns, t0, t1,
         steps_per_turn=24, sides=8):
    """the part from t0 to t1 (0 to 1) of a coil spring that starts at start
    and runs length along axis (a unit vector), swept by a circle"""
    helper = [1.0, 0.0, 0.0] if abs(axis[0]) < 0.9 else [0.0, 1.0, 0.0]
    u = unit(cross(axis, helper))
    v = cross(axis, u)

    def centre(t):
        a = 2.0 * math.pi * turns * t
        return [start[i] + length * t * axis[i] +
                coil_radius * (math.cos(a) * u[i] + math.sin(a) * v[i])
                for i in range(3)]

    count = max(2, int(steps_per_turn * turns * (t1 - t0)))
    ts = [t0 + (t1 - t0) * k / count for k in range(count + 1)]
    rings = []
    for t in ts:
        c = centre(t)
        tangent = unit(sub(centre(min(t + 1e-4, 1.0)), centre(max(t - 1e-4, 0.0))))
        n1 = unit(cross(tangent, axis))
        n2 = cross(tangent, n1)
        rings.append([(add(c, [wire * (math.cos(2 * math.pi * j / sides) * n1[i] +
                                       math.sin(2 * math.pi * j / sides) * n2[i])
                               for i in range(3)]),
                       [math.cos(2 * math.pi * j / sides) * n1[i] +
                        math.sin(2 * math.pi * j / sides) * n2[i]
                        for i in range(3)])
                      for j in range(sides)])
    for r0, r1 in zip(rings, rings[1:]):
        for j in range(sides):
            k = (j + 1) % sides
            m.triangle(material, [r0[j][0], r0[k][0], r1[k][0]],
                       [r0[j][1], r0[k][1], r1[k][1]])
            m.triangle(material, [r0[j][0], r1[k][0], r1[j][0]],
                       [r0[j][1], r1[k][1], r1[j][1]])


def telescope(name, parent, inner, outer, body, mass, geometry):
    """a universal joint at inner on parent (a frame) and a slide along the
    line from inner to outer: body is the part at inner and the returned
    virtual body slides along it to outer; its ball joint is added by the
    caller. Adds no constraint (as in the raptor example)"""
    along = sub(outer, inner)
    first = [1.0, 0.0, 0.0] if abs(along[0]) < abs(along[1]) \
        else [0.0, 1.0, 0.0]
    bodies.append({"name": body, "geometry": geometry,
                   "center_of_mass": r([0.5 * x for x in along]),
                   "inertia": inertia(mass, 0.05, 0.002, 0.05)})
    universal(name + "_inner", parent, [[body], "origin"], first, along)
    slide = name + "_slide"
    bodies.append({"name": slide, "geometry": [],
                   "center_of_mass": [0.0, 0.0, 0.0],
                   "inertia": inertia(0.5, 0.001, 0.001, 0.001)})
    joint(name + "_slide", "prismatic", [[body], "origin"],
          [[slide], "origin"], axis=r(unit(along), 9),
          initial_displacement=round(norm(along), 6), initial_velocity=0.0)
    return slide


def universal(name, parent, child, first, along):
    """universal joint: about first on the parent, then about the axis
    normal to first and to along (the direction of the child)"""
    v = new_virtual()
    second = unit(cross(along, first))
    joint(name + "_1", "revolute", parent, [[v], "origin"],
          axis=r(unit(first), 9))
    joint(name + "_2", "revolute", [[v], "origin"], child,
          axis=r(second, 9))


def tire(c):
    """the Iowa tire, with a relaxation length, on gravel"""
    force_modules.append({
        "type": "tire", "name": c + "tire", "tire_type": "iowa",
        "wheel_body": c + "wheel",
        "iowa": {"unloaded_radius": radius, "vertical_stiffness": 180000.0,
                 "vertical_damping": 1500.0, "friction_coefficient": 0.75,
                 "rolling_resistance": 0.025,
                 "cornering_stiffness": 35000.0,
                 "aligning_coefficient": 0.03, "relaxation_length": 0.3,
                 "segments": 20}})


# floating chassis: x, y, z, roll, pitch, yaw
chain = ["fixed"] + [new_virtual() for _ in range(5)] + ["chassis"]
for k, (kind, axis, q0) in enumerate([
        ("prismatic", [1, 0, 0], 0.0), ("prismatic", [0, 1, 0], 0),
        ("prismatic", [0, 0, 1], ride_drop), ("revolute", [1, 0, 0], 0),
        ("revolute", [0, 1, 0], 0), ("revolute", [0, 0, 1], 0)]):
    joint("chassis_" + "xyzXYZ"[k], kind, [[chain[k]], "origin"],
          [[chain[k + 1]], "origin"], axis=axis, initial_displacement=q0,
          initial_velocity=0.0)

# steering rack (not drawn), moved by a PID controller and a damper standing
# in for the steering column and the driver's hands on the wheel
chassis_frames.append({"name": "steering_rack", "T": r(rack_origin)})
bodies.append({"name": "steering_rack", "geometry": [],
               "frames": rack_frames, "center_of_mass": [0.0, 0.0, 0.0],
               "inertia": inertia(3.0, 0.08, 0.001, 0.08)})
joint("rack_slide", "prismatic", [["chassis"], "steering_rack"],
      [["steering_rack"], "origin"], axis=[0, 1, 0],
      initial_displacement=0.0, initial_velocity=0.0,
      damping_coefficients=[0.0, 4000.0])

for end, hp in (("front", front), ("rear", rear)):
    for side, s in (("right", 1.0), ("left", -1.0)):
        def m(p):
            return [p[0], s * p[1], p[2]] if isinstance(p, list) else p

        c = side + "_" + end + "_"
        h = {k: m(v) for k, v in hp.items()}
        knuckle = h["knuckle"]
        bodies += [
            {"name": c + "wheel", "geometry": mesh(c + "wheel"),
             "center_of_mass": [0.0, 0.0, 0.0],
             "inertia": inertia(masses["wheel"], 0.6, 1.1, 0.6)},
            {"name": c + "knuckle",
             "geometry": mesh(c + "knuckle") + drawn(c + "knuckle_parts"),
             "comment": "estimated: upright, hub, brake and the lower strut",
             "frames": [
                 {"name": "hub", "T": r(sub(h["wheel"], knuckle))},
                 {"name": "lower_ball_joint",
                  "T": r(sub(h["lower_ball_joint"], knuckle))},
                 {"name": "strut",
                  "T": r(sub(h["strut_bottom"], knuckle))},
                 {"name": "tie_rod",
                  "T": r(sub(h["tie_rod_outer"], knuckle))}],
             "center_of_mass": [0.0, 0.0, -0.05],
             "inertia": inertia(masses["knuckle"], 0.12, 0.1, 0.06)},
            {"name": c + "lower_arm", "geometry": drawn(c + "lower_arm"),
             "frames": [{"name": "ball_joint",
                         "T": r(sub(h["lower_ball_joint"], h["lower_arm"]))},
                        {"name": "drop_link",
                         "T": r(sub(h["link_bottom"], h["lower_arm"]))}],
             "center_of_mass": [0.0, s * 0.18, 0.01],
             "inertia": inertia(masses["lower_arm"], 0.06, 0.05, 0.1)},
            {"name": c + "tie_rod", "geometry": drawn(c + "tie_rod"),
             "frames": [{"name": "outer",
                         "T": r(sub(h["tie_rod_outer"],
                                    h["tie_rod_inner"]))}],
             "center_of_mass": r([0.5 * x for x in
                                  sub(h["tie_rod_outer"],
                                      h["tie_rod_inner"])]),
             "inertia": inertia(masses["tie_rod"], 0.01, 0.0002, 0.01)}]
        chassis_frames += [
            {"name": c + "lower_arm", "T": r(h["lower_arm"])},
            {"name": c + "strut", "T": r(h["strut_top"])}]

        joint(c + "lower_arm_pivot", "revolute",
              [["chassis"], c + "lower_arm"], [[c + "lower_arm"], "origin"],
              axis=[1, 0, 0])
        joint(c + "lower_ball_joint", "spherical",
              [[c + "lower_arm"], "ball_joint"],
              [[c + "knuckle"], "lower_ball_joint"])
        joint(c + "hub", "revolute", [[c + "knuckle"], "hub"],
              [[c + "wheel"], "origin"], axis=[0, 1, 0],
              initial_displacement=0.0, initial_velocity=0.0,
              damping_coefficients=[0.0, 0.3])

        # the strut: a ball joint at the top mount, then a slide along its
        # axis to the clamp on the knuckle, sprung and damped. The spring
        # pushes the ends apart with the preload at the modelled length
        along = sub(h["strut_bottom"], h["strut_top"])
        length = norm(along)
        strut = c + "strut"
        bodies.append({"name": strut, "geometry": drawn(strut),
                       "center_of_mass": r([0.4 * x for x in along]),
                       "inertia": inertia(masses["strut"], 0.03, 0.03,
                                          0.002)})
        joint(c + "strut_top", "spherical", [["chassis"], c + "strut"],
              [[strut], "origin"])
        joint(c + "strut", "prismatic", [[strut], "origin"],
              [[c + "knuckle"], "strut"], axis=r(unit(along), 9),
              initial_displacement=round(length, 6), initial_velocity=0.0,
              stiffness_coefficients=[
                  round(-h["preload"] - h["k"] * length, 1), h["k"]],
              damping_coefficients=[0.0, h["c"]])
        # the bump stop and the rebound stop, between the top mount and the
        # clamp on the knuckle, along the strut
        force_modules += [
            {"type": "spring_damper", "name": c + "bump_stop",
             "frames": [["chassis", c + "strut"], [c + "knuckle", "strut"]],
             "free_length": round(length, 6), "acts": "compression",
             "gap": BUMP_GAP, "force_curve": BUMP_CURVE, "damping": 1000.0,
             "smoothing": 0.005},
            {"type": "spring_damper", "name": c + "rebound_stop",
             "frames": [["chassis", c + "strut"], [c + "knuckle", "strut"]],
             "free_length": round(length, 6), "acts": "tension",
             "gap": REBOUND_GAP, "stiffness": 300000.0, "damping": 1000.0,
             "smoothing": 0.005}]

        # the anti-roll bar: each half turns on the body about y; its arm
        # carries the drop link, with a ball joint at the top and a universal
        # joint on the lower arm. A torsion spring between the halves (below)
        # is the twist of the bar
        half = c + "anti_roll_bar"
        arm_end = sub(h["link_top"], h["bar"])
        bodies.append({"name": half, "geometry": drawn(half),
                       "frames": [{"name": "drop_link", "T": r(arm_end)}],
                       "center_of_mass": [0.0, round(s * 0.25, 6), 0.0],
                       "inertia": inertia(masses["bar"], 0.03, 0.006, 0.03)})
        joint(half, "revolute", [["chassis"], end + "_anti_roll_bar"],
              [[half], "origin"], axis=[0, 1, 0])
        link = c + "drop_link"
        drop = sub(h["link_bottom"], h["link_top"])
        bodies.append({"name": link, "geometry": drawn(link),
                       "center_of_mass": r([0.5 * x for x in drop]),
                       "inertia": inertia(masses["drop_link"], 0.0006,
                                          0.0006, 0.00005)})
        joint(link + "_top", "spherical", [[half], "drop_link"],
              [[link], "origin"])
        lower = c + "drop_link_bottom"
        link_frames = [{"name": "bottom", "T": r(drop)}]
        bodies[-1]["frames"] = link_frames
        universal(lower, [[link], "bottom"], [[c + "lower_arm"], "drop_link"],
                  [1.0, 0.0, 0.0], drop)
        ab = part(half)
        bend = sub(h["bar_end"], h["bar"])
        bar(ab, "dark", [0.0, 0.0, 0.0], bend, h["bar_radius"])
        bar(ab, "dark", bend, arm_end, h["bar_radius"])
        ball(ab, "dark", bend, h["bar_radius"])
        ab.write()
        dl = part(link)
        bar(dl, "steel", [0.0, 0.0, 0.0], drop, 0.007)
        ball(dl, "dark", [0.0, 0.0, 0.0], 0.015)
        ball(dl, "dark", drop, 0.015)
        dl.write()

        # the tie rod: a ball joint at the rack (front) or the body (rear),
        # and a universal joint at the knuckle
        rod = sub(h["tie_rod_outer"], h["tie_rod_inner"])
        if end == "front":
            rack_frames.append({"name": c + "tie_rod",
                                "T": r(sub(h["tie_rod_inner"], rack_origin))})
            inner = [["steering_rack"], c + "tie_rod"]
        else:
            chassis_frames.append({"name": c + "tie_rod",
                                   "T": r(h["tie_rod_inner"])})
            inner = [["chassis"], c + "tie_rod"]
        joint(c + "tie_rod_inner", "spherical", inner,
              [[c + "tie_rod"], "origin"])
        universal(c + "tie_rod_outer", [[c + "tie_rod"], "outer"],
                  [[c + "knuckle"], "tie_rod"], [0.0, 0.0, 1.0], rod)
        tire(c)

        # the drawn parts of the corner, each in its body's coordinates
        def local(p, origin):
            return sub(p, origin)
        u = unit(along)
        down = [x for x in u]
        # lower arm: the bushings of its pivot, along x, and the arms to the
        # ball joint
        arm = part(c + "lower_arm")
        o = h["lower_arm"]
        ball_joint = local(h["lower_ball_joint"], o)
        for dx in (0.14, -0.16):
            pivot = [dx, 0.0, 0.0]
            mesh_builder.cylinder(arm, "dark", pivot, [1.0, 0.0, 0.0], 0.03,
                                  0.07, 16)
            bar(arm, "steel", pivot, ball_joint, 0.016)
        ball(arm, "dark", ball_joint, 0.03)
        arm.write()
        # strut, on its top mount: the piston rod, the top mount and the
        # upper spring seat, and the upper part of the spring
        top = part(strut)
        bar(top, "steel", [0.0, 0.0, 0.0], [0.33 * x for x in down], 0.011)
        mesh_builder.cylinder(top, "dark", [0.0, 0.0, 0.0], down, 0.06, 0.03)
        seat = [0.06 * x for x in down]
        mesh_builder.cylinder(top, "steel", seat, down, 0.07, 0.008)
        # the bump stop, on the rod above the damper tube
        stop_end = length - 0.22 - BUMP_GAP
        bar(top, "boot", [0.035 * x for x in down],
            [stop_end * x for x in down], 0.02)
        spring_length = length - 0.06 - 0.10
        coil(top, "spring", seat, down, 0.055, 0.007, spring_length, 5.0,
             0.0, 0.55)
        top.write()
        # knuckle: the upright from the hub to the strut and the ball joint,
        # the strut tube clamped in it, the lower spring seat on the tube,
        # and the lower part of the spring, which slides into the upper part
        # as the strut shortens
        kn = part(c + "knuckle_parts")
        clamp = local(h["strut_bottom"], knuckle)
        bar(kn, "dark", [0.0, 0.0, 0.0], clamp, 0.03)
        bar(kn, "dark", [0.0, 0.0, 0.0],
            local(h["lower_ball_joint"], knuckle), 0.025)
        bar(kn, "dark", [0.0, 0.0, 0.0], local(h["tie_rod_outer"], knuckle),
            0.015)
        mesh_builder.cylinder(kn, "dark", [0.0, 0.0, 0.0], [0.0, 1.0, 0.0],
                              0.07, 0.05, 20)
        tube_top = add(clamp, [-0.22 * x for x in down])
        bar(kn, "damper", add(clamp, [0.05 * x for x in down]), tube_top,
            0.028)
        lower_seat = add(clamp, [-0.10 * x for x in down])
        mesh_builder.cylinder(kn, "steel", lower_seat, down, 0.07, 0.008)
        coil(kn, "spring", add(lower_seat, [-spring_length * x for x in down]),
             down, 0.055, 0.007, spring_length, 5.0, 0.45, 1.0)
        kn.write()
        # tie rod, with its ends
        tr = part(c + "tie_rod")
        outer = local(h["tie_rod_outer"], h["tie_rod_inner"])
        bar(tr, "steel", [0.0, 0.0, 0.0], outer, 0.011)
        ball(tr, "dark", [0.0, 0.0, 0.0], 0.02)
        ball(tr, "dark", outer, 0.018)
        tr.write()

        # the halfshaft, drawn on a body that follows the suspension: a
        # universal joint at the differential, a slide along the shaft and a
        # ball joint at the hub. It adds no constraint and carries no torque
        # (the powertrain drives the hub joints)
        shaft_inner = [h["knuckle"][0], s * halfshaft_inner_y, halfshaft_z]
        shaft_outer = h["knuckle"]
        hs = part(c + "halfshaft")
        shaft = sub(shaft_outer, shaft_inner)
        bar(hs, "steel", [0.0, 0.0, 0.0], shaft, 0.014)
        ball(hs, "boot", [0.0, 0.0, 0.0], 0.04)
        ball(hs, "boot", [0.85 * x for x in shaft], 0.04)
        hs.write()
        chassis_frames.append({"name": c + "halfshaft",
                               "T": r(shaft_inner)})
        slide = telescope(c + "halfshaft", [["chassis"], c + "halfshaft"],
                          shaft_inner, shaft_outer, c + "halfshaft", 3.0,
                          drawn(c + "halfshaft"))
        joint(c + "halfshaft_outer", "spherical", [[slide], "origin"],
              [[c + "knuckle"], "origin"])

# the twist of each anti-roll bar: a torsion spring between its halves
for end, hp in (("front", front), ("rear", rear)):
    chassis_frames.append({"name": end + "_anti_roll_bar",
                           "T": r(hp["bar"])})
    force_modules.append({
        "type": "spring_damper", "name": end + "_anti_roll_bar",
        "axis": [0.0, 1.0, 0.0],
        "frames": [["right_" + end + "_anti_roll_bar", "origin"],
                   ["left_" + end + "_anti_roll_bar", "origin"]],
        "stiffness": hp["bar_stiffness"], "damping": 2.0})

# the steering rack, its housing on the body, the differentials and the prop
# shaft
rack = part("steering_rack")
bar(rack, "steel", [0.0, -0.30, 0.0], [0.0, 0.30, 0.0], 0.012)
rack.write()
for b in bodies:
    if b["name"] == "steering_rack":
        b["geometry"] = drawn("steering_rack")
body_parts = part("chassis_parts")
bar(body_parts, "dark", add(rack_origin, [0.0, -0.22, 0.0]),
    add(rack_origin, [0.0, 0.22, 0.0]), 0.025)
for x in (front["knuckle"][0], rear["knuckle"][0]):
    mesh_builder.cylinder(body_parts, "dark", [x, 0.0, halfshaft_z],
                          [0.0, 1.0, 0.0], 0.10, 2 * halfshaft_inner_y - 0.04,
                          24)
for hp in (front, rear):  # the bushings of the anti-roll bars
    for y in (-0.36, 0.36):
        mesh_builder.cylinder(body_parts, "boot",
                              add(hp["bar"], [0.0, y, 0.0]), [0.0, 1.0, 0.0],
                              hp["bar_radius"] + 0.01, 0.04, 16)
bar(body_parts, "steel", [front["knuckle"][0] - 0.1, 0.0, halfshaft_z],
    [rear["knuckle"][0] + 0.1, 0.0, halfshaft_z], 0.03)
body_parts.write()

# the lenses of the stop lamps, which light up as the car brakes: cut from the
# glow of the brake lights of the source (brake_lamps.glb, written by the
# split script), a few quads over the lamps textured with lit lamps. The
# pupil of each tail lamp, the square in the middle of its glow, and the high
# mounted stop lamp on the spoiler are kept, as pixel rectangles of the 512 x
# 512 texture (u from the left, v from the top); the reflectors low on the
# bumper are not lamps. The lenses lie 2 mm proud of the tail lamps' own
# glow, which is the same picture, so that the two do not fight over pixels
LENS_RECTS = {"pupil": (382.0, 425.0, 428.0, 457.0),
              "high": (0.0, 135.0, 512.0, 195.0)}
LENS_OFFSET = 0.002


def read_glb(path):
    """the positions, texture coordinates and triangles of the one mesh of a
    binary glTF file, with its texture image (bytes, mime type)"""
    with open(path, "rb") as f:
        data = f.read()
    length = struct.unpack("<I", data[12:16])[0]
    gltf = json.loads(data[20:20 + length])
    binary = data[20 + length + 8:]

    def view(i):
        v = gltf["bufferViews"][i]
        start = v.get("byteOffset", 0)
        return binary[start:start + v["byteLength"]]

    def accessor(i):
        a = gltf["accessors"][i]
        size = {"SCALAR": 1, "VEC2": 2, "VEC3": 3}[a["type"]]
        dtype = {5126: np.float32, 5123: np.uint16, 5125: np.uint32}[
            a["componentType"]]
        raw = view(a["bufferView"])[a.get("byteOffset", 0):]
        return np.frombuffer(raw, dtype, a["count"] * size).reshape(
            a["count"], size)
    primitive = gltf["meshes"][0]["primitives"][0]
    image = gltf["images"][0]
    return (accessor(primitive["attributes"]["POSITION"]).astype(float),
            accessor(primitive["attributes"]["TEXCOORD_0"]).astype(float),
            accessor(primitive["indices"]).reshape(-1, 3),
            view(image["bufferView"]), image["mimeType"])


def clip(polygon, axis, value, keep_above):
    """the part of polygon, a list of (position, uv in pixels), on one side
    of the line uv[axis] = value (Sutherland-Hodgman)"""
    def inside(q):
        return (q[1][axis] >= value) == keep_above
    out = []
    for k in range(len(polygon)):
        a, b = polygon[k - 1], polygon[k]
        if inside(b) != inside(a):
            t = (value - a[1][axis]) / (b[1][axis] - a[1][axis])
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
        if inside(b):
            out.append(b)
    return out


def write_lenses():
    """writes the lenses of the stop lamps, <side>_stop_lamp_lens.obj and
    high_stop_lamp_lens.obj with polo_r_wrc_lamp_glow.jpg, from
    brake_lamps.glb; returns the middle of the right pupil and its outward
    normal"""
    positions, uvs, triangles, image, mime = read_glb(
        os.path.join(HERE, "brake_lamps.glb"))
    assert mime == "image/jpeg"
    with open(os.path.join(HERE, "polo_r_wrc_lamp_glow.jpg"), "wb") as f:
        f.write(image)
    lenses = {"right_stop_lamp_lens": [], "left_stop_lamp_lens": [],
              "high_stop_lamp_lens": []}
    for triangle in triangles:
        p, t = positions[triangle], uvs[triangle] * 512.0
        normal = np.cross(p[1] - p[0], p[2] - p[0])
        if np.linalg.norm(normal) < 1e-12:
            continue
        normal /= np.linalg.norm(normal)
        if normal[0] > 0.0:  # the lamps face back
            normal = -normal
        for kind, (u0, v0, u1, v1) in LENS_RECTS.items():
            polygon = list(zip(p, t))
            for axis, value, above in ((0, u0, True), (0, u1, False),
                                       (1, v0, True), (1, v1, False)):
                polygon = clip(polygon, axis, value, above)
                if not polygon:
                    break
            if len(polygon) < 3:
                continue
            name = ("high" if kind == "high" else
                    "right" if p[:, 1].mean() > 0.0 else "left") + \
                "_stop_lamp_lens"
            lenses[name].append(([q[0] + LENS_OFFSET * normal
                                  for q in polygon],
                                 [q[1] for q in polygon], normal))
    middle = None
    for name, polygons in lenses.items():
        points, coordinates, normals, faces = [], [], [], []
        for polygon, uv, normal in polygons:
            first = len(points)
            points += polygon
            # OBJ texture coordinates count v from the bottom
            coordinates += [(q[0] / 512.0, 1.0 - q[1] / 512.0) for q in uv]
            normals += [normal] * len(polygon)
            faces += [(first, first + k, first + k + 1)
                      for k in range(1, len(polygon) - 1)]
        assert faces, name
        mesh_builder.textured_mesh(HERE, name, points, coordinates, normals,
                                   faces, "polo_r_wrc_lamp_glow.jpg",
                                   roughness=0.2)
        if name == "right_stop_lamp_lens":
            n = np.mean(normals, axis=0)
            middle = (np.mean(points, axis=0), n / np.linalg.norm(n))
    return middle


lens_middle, lens_normal = write_lenses()

# the lamps: IES lights on the chassis, at the headlamps and the tail lamps of
# the body mesh (the middle of each lamp's glow, just ahead of its lens). An
# IES light shines along -z of its frame (the nadir of the photometry), with
# horizontal angle 0 along x; here x points up, so horizontal angle 0 is up
# and 90 is to the car's left at the front, to its right at the back. The
# photometry is not measured: write_ies() draws it to the photometric points
# of UN ECE regulations 112 (low beam) and 7 (rear position lamp)
LAMPS = {
    "headlamp": {"T": [1.68, 0.61, -0.59], "file": "polo_r_wrc_low_beam.ies",
                 "color": [1.0, 0.97, 0.92],
                 # rows of R: the frame's z points back, x up, y left
                 "R": [[0.0, 0.0, -1.0], [0.0, -1.0, 0.0], [-1.0, 0.0, 0.0]]},
    "tail_lamp": {"T": [-1.72, 0.58, -0.75],
                  "file": "polo_r_wrc_tail_lamp.ies",
                  "color": [1.0, 0.04, 0.02],
                  # the frame's z points forward, x up, y right
                  "R": [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]},
    # the stop lamps, at the pupils of the tail lamps (the middle of the
    # right lens, 2 cm out from it). They are lights of the chassis body, so
    # that the map "brake_lights" can set their intensity from the brake
    # pedal; they start dark, and their lenses glow with them
    "brake_lamp": {"T": r(list(lens_middle + 0.02 * lens_normal), 3),
                   "file": "polo_r_wrc_stop_lamp.ies",
                   "color": [1.0, 0.02, 0.01], "body": True,
                   "R": [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]}}
# illuminance (lux) that shows a white, matte surface as white: the scale of
# eyes adapted to the dark, not of daylight, so that the beams show on the
# ground although the viewer's sky stays lit
WHITE_LUX = 10.0


def low_beam(elevation, azimuth):
    """Luminous intensity (cd) of a low beam for traffic on the right, from
    the elevation (up) and the azimuth (to the left) in degrees: a hot spot
    1.5 degrees down and 2 degrees right, about 29,000 cd with the wide
    spread for the foreground, and the cutoff of the regulation, flat at
    0.57 degrees down on the left and rising at 15 degrees on the right,
    above which only a glare of a few hundred candela is allowed."""
    cutoff = -0.57 + (min(-azimuth * math.tan(math.radians(15.0)), 1.5)
                      if azimuth < 0.0 else 0.0)
    if elevation > cutoff:
        return 300.0 * math.exp(-0.5 * ((elevation - cutoff) / 3.0) ** 2) * \
            math.exp(-0.5 * (azimuth / 30.0) ** 2)
    below = cutoff - elevation
    hot = math.exp(-0.5 * ((azimuth + 2.0) / 7.0) ** 2) * \
        math.exp(-0.5 * ((elevation + 1.5) / (1.5 if elevation > -1.5
                                              else 3.5)) ** 2)
    spread = math.exp(-0.5 * (azimuth / 28.0) ** 2) * \
        math.exp(-0.5 * ((elevation + 2.5) / 6.0) ** 2)
    rise = min(below / 0.3, 1.0)  # the edge of the cutoff over 0.3 degrees
    return rise * (25000.0 * hot + 4000.0 * spread) + (1.0 - rise) * 300.0


def tail_lamp(elevation, azimuth):
    """Luminous intensity (cd) of a rear position lamp: 10 cd on its axis
    (the regulation allows 4 to 17), falling to half at about 53 degrees to
    the side and 30 degrees up or down: the lens spreads it widely, and some
    of it reaches the ground behind the car."""
    return 10.0 * math.exp(-0.5 * (azimuth / 45.0) ** 2) * \
        math.exp(-0.5 * (elevation / 25.0) ** 2)


def stop_lamp(elevation, azimuth):
    """Luminous intensity (cd) of a stop lamp: 80 cd on its axis (the
    regulation allows 60 to 185 for a lamp of one level), falling to half at
    about 41 degrees to the side and 21 degrees up or down."""
    return 80.0 * math.exp(-0.5 * (azimuth / 35.0) ** 2) * \
        math.exp(-0.5 * (elevation / 18.0) ** 2)


def ies_table(intensity):
    """Type C photometry: vertical angles from the nadir, the lamp's axis, to
    90 degrees (no light backwards), horizontal angles all round, 0 up and 90
    to the side, and the candela over the vertical angles for each
    horizontal angle; intensity(elevation, azimuth) in degrees gives the
    candela."""
    vertical = [0.25 * i for i in range(8)] + \
        [2.0 + 0.5 * i for i in range(17)] + list(range(11, 91))
    horizontal = [5.0 * i for i in range(73)]
    candela = []
    for h in horizontal:
        for v in vertical:
            t, p = math.radians(v), math.radians(h)
            # the direction: cos(t) along the axis, sin(t) towards the
            # horizontal angle (0 up, 90 to the side)
            up, side, ahead = (math.sin(t) * math.cos(p),
                               math.sin(t) * math.sin(p), math.cos(t))
            candela.append(round(intensity(
                math.degrees(math.asin(max(-1.0, min(1.0, up)))),
                math.degrees(math.atan2(side, ahead))), 1))
    return vertical, horizontal, candela


def write_ies(path, title, intensity):
    """Writes the photometry of ies_table(intensity) as an IES LM-63-2002
    file."""
    vertical, horizontal, candela = ies_table(intensity)
    lines = ["IESNA:LM-63-2002", "[TEST] none: drawn to the photometric "
             "points of the regulation, not measured",
             "[MANUFAC] shift example polo_r_wrc", "[LUMINAIRE] " + title,
             "TILT=NONE",
             # 1 lamp, absolute photometry (-1), multiplier 1, the angle
             # counts, type C (1), meters (2), size, ballast factors, watts
             "1 -1 1 %d %d 1 2 0.15 0.15 0.05" % (len(vertical),
                                                  len(horizontal)),
             "1 1 55"]

    def numbers(values):
        return [" ".join("%g" % v for v in values[i:i + 10])
                for i in range(0, len(values), 10)]
    lines += numbers(vertical) + numbers(horizontal)
    for i in range(len(horizontal)):
        lines += numbers(candela[i * len(vertical):(i + 1) * len(vertical)])
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


PHOTOMETRY = {"headlamp": low_beam, "tail_lamp": tail_lamp,
              "brake_lamp": stop_lamp}
lights, chassis_lights, brake_scale = [], [], 1.0
for lamp, p in LAMPS.items():
    # the brightest candela of the file, the lamp's brightness at Kd
    peak = max(ies_table(PHOTOMETRY[lamp])[2])
    for side, sign in (("right", 1.0), ("left", -1.0)):
        name = side + "_" + lamp
        # not mirrored: both headlamps have the cutoff of traffic on the
        # right
        chassis_frames.append({"name": name, "T": r([p["T"][0],
                                                     sign * p["T"][1],
                                                     p["T"][2]]),
                               "R": p["R"]})
        light = {"name": name, "type": "ies",
                 "parent": ["chassis", name], "filename": p["file"],
                 "Ka": [0.0, 0.0, 0.0], "Kd": p["color"],
                 "Ks": p["color"], "attenuation": [0.0, 0.0, 1.0],
                 "intensity": round(peak / WHITE_LUX, 4)}
        if p.get("body"):
            brake_scale = light["intensity"]
            light["intensity"] = 0.0
            # the lens of the lamp, and on the right the high mounted one
            light["glow"] = [side + "_stop_lamp_lens"] + \
                (["high_stop_lamp_lens"] if side == "right" else [])
            chassis_lights.append(light)
        else:
            lights.append(light)

bodies.insert(0, {
    "name": "chassis",
    "geometry": mesh("chassis") + drawn("chassis_parts") +
    [{"type": "mesh", "name": lens, "filename": [lens + ".obj"]}
     for lens in ("right_stop_lamp_lens", "left_stop_lamp_lens",
                  "high_stop_lamp_lens")],
    "frames": chassis_frames, "lights": chassis_lights,
    "center_of_mass": r(chassis_com),
    "comment": "estimated: body shell, roll cage, engine, transmission, "
               "crew and fuel",
    "inertia": inertia(masses["chassis"], 420.0, 1600.0, 1750.0)})

# the drivetrain. The ratios are estimates: a close ratio sequential box over
# a final drive of 3.5, sixth reaching about 190 km/h at 7,500 rpm and first
# about 70 km/h, as on a gravel stage. The clutch and the gears' clutches are
# smoothed so that they do not set too short a step of the integrator (see
# the raptor example): they slip by a few rad/s at full torque. So is the
# friction of the limited slip differentials, over 2 rad/s of slip between
# the wheels: at the default 0.5 rad/s it is the fastest motion of the car
# (shift-cli --linearize), and the steps are a third as long
force_modules.append({
    "type": "powertrain", "name": "drive",
    "comment": "published: 232 kW at 6,250 rpm, 425 Nm at 5,000 rpm; the "
               "rest estimated",
    "engine": {"inertia": 0.12, "idle_speed": 110.0, "high_idle_speed": 890.0,
               "torque_curve": [110, 150, 300, 330, 420, 410, 524, 425,
                                654, 355, 785, 280, 890, 0],
               "friction_curve": [110, -15, 890, -60]},
    "coupling": {"type": "clutch", "capacity": 900.0, "slip_speed": 40.0,
                 "engagement": 0.0},
    "transmission": {"ratios": [3.60, 2.96, 2.43, 2.00, 1.65, 1.35],
                     "reverse_ratio": -3.60, "gear": 1,
                     "clutch_capacity": 1500.0, "slip_speed": 50.0,
                     "input_inertia": 0.3, "output_inertia": 1.5,
                     "shift_time": 0.08, "spin_loss": 0.05},
    "axles": [{"name": end, "left": "left_" + end + "_hub",
               "right": "right_" + end + "_hub", "direction": -1,
               "final_drive": 3.5, "shaft_stiffness": 12000.0,
               "shaft_damping": 40.0, "differential": "limited_slip",
               "bias_torque": 80.0, "bias_ratio": bias, "slip_speed": 2.0,
               "efficiency": 0.96}
              for end, bias in (("front", 0.25), ("rear", 0.35))],
    "brakes": [{"name": side + "_" + end, "joint": side + "_" + end + "_hub",
                "direction": -1, "capacity": capacity}
               for side in ("left", "right")
               for end, capacity in (("front", 1500.0), ("rear", 600.0))]})

controllers.append({
    "type": "pid", "name": "steering", "kp": 1000000.0, "ki": 2000000.0,
    "kd": 0.0, "target": 0.0, "sensor": "rack_position",
    "joint": "rack_slide"})


def write_terrain():
    """Writes polo_r_wrc_stage.tiff, the ground of the stage.

    Fixed waves, so the ground is the same every time: hills, crests and
    dips, and short waves at angles to each other, the bumps and ruts of a
    gravel road; and the jumps of TERRAIN["crests"].
    The ground is flat around the start, where the car is placed.
    """
    x = TERRAIN["origin"][0] + TERRAIN["spacing"] * np.arange(TERRAIN["size"][0])
    y = TERRAIN["origin"][1] + TERRAIN["spacing"] * np.arange(TERRAIN["size"][1])
    X, Y = np.meshgrid(x, y)
    elevation = np.zeros_like(X)
    for amplitude, wavelength, heading, phase in TERRAIN["waves"]:
        along = (X * np.cos(np.radians(heading)) +
                 Y * np.sin(np.radians(heading)))
        elevation += amplitude * np.sin(2.0 * np.pi * along / wavelength + phase)
    flat, full = TERRAIN["flat_radius"], TERRAIN["full_radius"]
    blend = np.clip((np.hypot(X, Y) - flat) / (full - flat), 0.0, 1.0)
    elevation *= blend * blend * (3.0 - 2.0 * blend)
    # the crests: a cosine bump along the heading, across a road of the
    # width, its edges rounded off over a quarter of the width, on top of the
    # waves
    for cx, cy, heading, height, length, width in TERRAIN["crests"]:
        c, s = np.cos(np.radians(heading)), np.sin(np.radians(heading))
        along = (X - cx) * c + (Y - cy) * s
        across = -(X - cx) * s + (Y - cy) * c
        bump = np.where(np.abs(along) < length / 2.0,
                        0.5 * (1.0 + np.cos(2.0 * np.pi * along / length)),
                        0.0)
        edge = np.clip((width / 2.0 - np.abs(across)) / (width / 4.0), 0.0,
                       1.0)
        elevation += height * bump * edge * edge * (3.0 - 2.0 * edge)
    # a sample is the z of the surface, and +z is down
    Image.fromarray((-elevation).astype(np.float32), mode="F").save(
        "polo_r_wrc_stage.tiff")
    print("wrote polo_r_wrc_stage.tiff: %d x %d samples, elevation %.2f .. "
          "%.2f m" % (len(x), len(y), elevation.min(), elevation.max()))


def schedule(name, interpolation, points, targets, **extra):
    controllers.append({"type": "schedule", "name": name,
                        "interpolation": interpolation, "points": points,
                        "targets": targets, **extra})


schedule("throttle", "linear", ROUTE["throttle"],
         ["force_modules/drive/engine/throttle"])
schedule("clutch_pedal", "linear", ROUTE["clutch"],
         ["force_modules/drive/coupling/engagement"])
schedule("gears", "step", ROUTE["gears"],
         ["force_modules/drive/transmission/gear"])
schedule("brake_pedal", "linear", ROUTE["brakes"],
         ["force_modules/drive/brakes/" + side + "_" + end + "/pressure"
          for side in ("left", "right") for end in ("front", "rear")])
schedule("steering_wheel", "cubic", ROUTE["rack"],
         ["controllers/steering/target"])
# the stop lamps light at full brightness once the pedal is pressed 5%
controllers.append({
    "type": "map", "name": "brake_lights", "component": "brake_pedal",
    "output": "value", "points": [0.0, 0.0, 0.05, brake_scale],
    "targets": ["bodies/chassis/lights/" + side + "_brake_lamp/intensity"
                for side in ("left", "right")]})

model = {
    "model_parameters": {
        "version": "0.2",
        "units": {"angle": "radians", "length": "meters",
                  "mass": "kilograms", "time": "seconds"},
        "gravity": [0.0, 0.0, 9.81],
        "include_directories": ["."],
        "solvers": [{"type": "rnca", "infinity_norm_multiplier": 2.0}],
        "integrators": [{"type": "ode113", "time_span": ROUTE["time_span"],
                         "time_step": 0.02, "relative_error": 0.0001,
                         "absolute_error": 0.0001, "omega_n": 50.0}],
        "loggers": [{"type": "csv", "csv_logfile": "polo_r_wrc.csv",
                     "timestamp_logfile": False}]},
    "frames": [
        {"name": "overhead_1", "T": [8.0, -8.0, -8.0]},
        {"name": "overhead_2", "T": [-8.0, -8.0, -8.0]},
        {"name": "terrain", "T": TERRAIN["origin"] + [0.0]}],
    "lights": [
        {"name": "overhead_1", "type": "point",
         "parent": ["fixed", "overhead_1"], "Ka": [0.4, 0.4, 0.4],
         "Kd": [0.6, 0.6, 0.6], "Ks": [0.2, 0.2, 0.2]},
        {"name": "overhead_2", "type": "point",
         "parent": ["fixed", "overhead_2"], "Ka": [0.6, 0.6, 0.6],
         "Kd": [0.6, 0.6, 0.6], "Ks": [0.2, 0.2, 0.2]}] + lights,
    "terrain": [{
        "type": "basic", "name": "gravel_stage",
        "filename": "polo_r_wrc_stage.tiff", "parent": [["fixed"], "terrain"],
        "x_scale": TERRAIN["spacing"], "y_scale": TERRAIN["spacing"],
        "coordinate_type": "sae", "interpolation_type": "bicubic",
        # crushed gravel of 2 to 4 cm: the photograph is about 16 stones
        # across, so a tile of 0.5 m
        "texture_filename": "qrc:/textures/pebbles.jpg",
        "normal_map_filename": "qrc:/textures/pebbles_normal.jpg",
        "texture_scale": 0.5,
        # the tiles shifted at random, so that they do not show, and patches
        # of packed dirt and of grass among the gravel
        "texture_variation": True,
        "surfaces": [
            {"texture_filename": "qrc:/textures/dry_sand.jpg",
             "normal_map_filename": "qrc:/textures/dry_sand_normal.jpg",
             "normal_scale": 0.4, "texture_scale": 2.0, "coverage": 0.35,
             "patch_size": 8.0},
            {"texture_filename": "qrc:/textures/grass.jpg",
             "normal_map_filename": "qrc:/textures/grass_normal.jpg",
             "texture_scale": 2.0, "coverage": 0.15, "patch_size": 25.0}]}],
    "assemblies": [{
        "name": "polo_r_wrc", "active": True,
        "force_modules": force_modules,
        "sensors": [{"type": "joint", "name": "rack_position",
                     "joint": "rack_slide", "quantity": "position"}],
        "controllers": controllers, "bodies": bodies, "joints": joints}]}
with open(out, "w") as f:
    json.dump(model, f, indent=2)
# the same car without its body shell, to watch the suspension work: only
# the parts added here are drawn on the chassis
bare = json.loads(json.dumps(model))
bare["model_parameters"]["loggers"][0]["csv_logfile"] = \
    "polo_r_wrc_suspension.csv"
for b in bare["assemblies"][0]["bodies"]:
    if b["name"] == "chassis":
        # the parts, and the lenses that glow with the stop lamps
        b["geometry"] = [g for g in b["geometry"]
                         if g["filename"] != ["chassis.glb"]]
with open(os.path.join(os.path.dirname(os.path.abspath(out)),
                       "polo_r_wrc_suspension.json"), "w") as f:
    json.dump(bare, f, indent=2)
os.chdir(os.path.dirname(os.path.abspath(out)))
write_terrain()
write_ies(LAMPS["headlamp"]["file"], "low beam headlamp", low_beam)
write_ies(LAMPS["tail_lamp"]["file"], "rear position lamp", tail_lamp)
write_ies(LAMPS["brake_lamp"]["file"], "stop lamp", stop_lamp)
# the license plate: the image that replaces the source's plate texture when
# the body is split (see "images" in the split configuration). The source's
# plate mesh shows the rectangle from pixel (12, 12) to (499, 109) of its
# 512 x 512 texture, upside down; the rest is the plate's dark holder
plate = Image.new("RGB", (512, 512), (31, 31, 31))
plate.paste(ImageOps.flip(mesh_builder.license_plate(
    (488, 98), "SHIFT", band=(0, 51, 153))), (12, 12))
plate.save("polo_r_wrc_plate.png")
print("bodies", len(bodies), "joints", len(joints))
