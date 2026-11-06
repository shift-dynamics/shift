"""writes raptor.json: the 2024 Ford F-150 Raptor R on its own suspension.

Text generation only. The hardpoints were measured from the meshes of the
source model, and the mass properties, springs and dampers are estimates.
The meshes come from shift/scripts/vehicle_splits/raptor.json (see
readme.rst).

Vehicle (SAE) coordinates: x forward, y right, z down, in meters, with the
origin on the ground. Hardpoints are given for the right side; the left side
is their mirror image in y.

Front, each side: a double wishbone. The lower and upper arms turn on the
frame and carry the knuckle on ball joints. A coil-over stands on the lower
arm, with a universal joint at its top mount. The knuckle is steered by a tie
rod from the rack (the source model has neither; they are not drawn).

Rear: a live axle, located by a lower link on each side and a Panhard rod,
which leave it free to rise, roll and pitch. Its upper links are stiff
springs along their length (their rubber bushings), so they hold its pitch:
as rigid links, the four parallel links would allow no roll (the real axle
rolls on its bushings). A coil-over on each side carries it.

The front halfshafts and the prop shaft telescope too: each is a universal
joint at its inner end, a slide along the shaft and a ball joint at the other
end, which adds no constraint. The truck drives on its rear wheels.
"""
import json
import math
import sys

out = sys.argv[1] if len(sys.argv) > 1 else "raptor.json"
speed = 15.0
radius = 0.394
# the meshes put the wheel centers 0.079 m above where the tires touch the
# ground: the chassis starts that much lower, and a little more for the
# tires to carry the truck
ride_drop = 0.10


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


# right side hardpoints, measured from the meshes; the body origins match
# the "bodies" of the split configuration
front = {
    "wheel": [2.017, 0.936, -0.473],
    "knuckle": [2.017, 0.8, -0.473],
    "lower_arm": [2.072, 0.373, -0.408],     # inner pivot, along x
    "upper_arm": [2.0335, 0.474, -0.834],    # inner pivot, along the axis
    "upper_arm_axis": [0.297, 0.0, -0.042],
    "lower_ball_joint": [2.075, 0.875, -0.31],
    "upper_ball_joint": [1.975, 0.745, -0.826],
    "coil_over_top": [2.024, 0.504, -0.928],
    "coil_over_bottom": [2.094, 0.7, -0.377],
    "halfshaft_inner": [1.999, 0.411, -0.492],
    "halfshaft_outer": [2.017, 0.76, -0.476],
    # tie rod: the outer end at the steering arm of the knuckle; the inner
    # end on the line through the inner pivots of the arms (front view),
    # for little bump steer
    "tie_rod_inner": [2.15, 0.381, -0.445],
    "tie_rod_outer": [2.147, 0.875, -0.445],
    # coil-over: rate, damping and force at the modelled length. Wheel rate
    # about 41 kN/m (1.3 Hz) with a motion ratio of about 0.59 at the coil-
    # over; the preload holds the modelled length with the truck at rest
    "k": 118000.0, "c": 10000.0, "preload": 10990.0,
}
rear = {
    "wheel": [-1.683, 0.936, -0.463],
    "lower_link_front": [-0.676, 0.543, -0.339],
    "lower_link_rear": [-1.601, 0.493, -0.355],
    "upper_link_front": [-0.824, 0.543, -0.453],
    "upper_link_rear": [-1.686, 0.573, -0.585],
    "coil_over_top": [-1.352, 0.675, -0.766],
    "coil_over_bottom": [-1.635, 0.694, -0.301],
    # wheel rate about 41 kN/m (1.5 Hz) with a motion ratio of 0.85; the
    # preload holds the modelled length with the truck at rest
    "k": 56000.0, "c": 4200.0, "preload": 5690.0,
}
axle_origin = [-1.683, 0.0, -0.463]
panhard_chassis = [-1.888, 0.433, -0.433]   # right, on the frame
panhard_axle = [-1.842, -0.394, -0.456]     # left, on the axle
prop_front = [0.393, 0.034, -0.551]
prop_rear = [-1.30, 0.03, -0.452]
rack_origin = [2.15, 0.0, -0.445]

bodies, joints, force_modules, controllers = [], [], [], []
chassis_frames, rack_frames, axle_frames = [], [], []
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


def across(along):
    """x or y, whichever is nearer normal to along"""
    return [1.0, 0.0, 0.0] if abs(along[0]) < abs(along[1]) \
        else [0.0, 1.0, 0.0]


def universal(name, parent, child, first, along):
    """universal joint: about first on the parent, then about the axis
    normal to first and to along (the direction of the child)"""
    v = new_virtual()
    second = unit(cross(along, first))
    joint(name + "_1", "revolute", parent, [[v], "origin"],
          axis=r(unit(first), 9))
    joint(name + "_2", "revolute", [[v], "origin"], child,
          axis=r(second, 9))


def telescope(name, parent, inner, outer, body, mass, **spring):
    """a universal joint at inner on parent (a frame) and a slide along the
    line from inner to outer: body is the part at inner and the returned
    virtual body slides along it to outer; its ball joint is added by the
    caller. Adds no constraint. spring: stiffness and damping coefficients
    of the slide (none: it slides freely)"""
    along = sub(outer, inner)
    universal(name + "_inner", parent, [[body], "origin"], across(along),
              along)
    slide = name + "_slide"
    bodies.append({"name": slide, "geometry": [],
                   "center_of_mass": [0.0, 0.0, 0.0],
                   "inertia": inertia(mass, 0.001, 0.001, 0.001)})
    joint(name + "_slide", "prismatic", [[body], "origin"],
          [[slide], "origin"], axis=r(unit(along), 9),
          initial_displacement=round(norm(along), 6), initial_velocity=0.0,
          **spring)
    return slide


def coil_over(name, top_frame, lower_frame, top, bottom, h, upper_mesh,
              lower_mesh, masses):
    """coil-over from top (a frame on the chassis) to bottom (a frame on
    the arm or axle): a universal joint at the top, a spring and damper
    along the axis and a ball joint at the bottom. The spring pushes the ends
    apart with the preload at the modelled length."""
    along = sub(bottom, top)
    length = norm(along)
    u = unit(along)
    upper, lower = name + "_upper", name + "_lower"
    bodies.append({"name": upper,
                   "geometry": mesh(upper_mesh) if upper_mesh else [],
                   "center_of_mass": r([0.3 * length * x for x in u]),
                   "inertia": inertia(masses[0], 0.05, 0.05, 0.01)})
    bodies.append({"name": lower,
                   "geometry": mesh(lower_mesh) if lower_mesh else [],
                   "center_of_mass": r([-0.2 * length * x for x in u]),
                   "inertia": inertia(masses[1], 0.03, 0.03, 0.005)})
    universal(name + "_top", top_frame, [[upper], "origin"],
              across(along), along)
    joint(name, "prismatic", [[upper], "origin"], [[lower], "origin"],
          axis=r(u, 9), initial_displacement=round(length, 6),
          initial_velocity=0.0,
          stiffness_coefficients=[round(-h["preload"] - h["k"] * length, 1),
                                  h["k"]],
          damping_coefficients=[0.0, h["c"]])
    joint(name + "_bottom", "spherical", [[lower], "origin"], lower_frame)


# floating chassis: x, y, z, roll, pitch, yaw
chain = ["fixed"] + [new_virtual() for _ in range(5)] + ["chassis"]
for k, (kind, axis, q0, dq0) in enumerate([
        ("prismatic", [1, 0, 0], 0.0, speed), ("prismatic", [0, 1, 0], 0, 0),
        ("prismatic", [0, 0, 1], ride_drop, 0), ("revolute", [1, 0, 0], 0, 0),
        ("revolute", [0, 1, 0], 0, 0), ("revolute", [0, 0, 1], 0, 0)]):
    joint("chassis_" + "xyzXYZ"[k], kind, [[chain[k]], "origin"],
          [[chain[k + 1]], "origin"], axis=axis, initial_displacement=q0,
          initial_velocity=dq0)

# steering rack (not drawn), held centered by a spring and damper standing
# in for the steering column and the driver's hands on the wheel
chassis_frames.append({"name": "steering_rack", "T": r(rack_origin)})
bodies.append({"name": "steering_rack", "geometry": [],
               "frames": rack_frames, "center_of_mass": [0.0, 0.0, 0.0],
               "inertia": inertia(5.0, 0.15, 0.002, 0.15)})
joint("rack_slide", "prismatic", [["chassis"], "steering_rack"],
      [["steering_rack"], "origin"], axis=[0, 1, 0],
      initial_displacement=0.0, initial_velocity=0.0,
      stiffness_coefficients=[0.0, 300000.0],
      damping_coefficients=[0.0, 6000.0])

for side, s in (("right", 1.0), ("left", -1.0)):
    def m(p):
        return [p[0], s * p[1], p[2]] if isinstance(p, list) else p

    # front
    c = side + "_front_"
    h = {k: m(v) for k, v in front.items()}
    h["upper_arm_axis"] = front["upper_arm_axis"]
    knuckle, lower_arm, upper_arm = h["knuckle"], h["lower_arm"], \
        h["upper_arm"]
    bodies += [
        {"name": c + "wheel", "geometry": mesh(c + "wheel"),
         "center_of_mass": [0.0, 0.0, 0.0],
         "inertia": inertia(50.0, 2.7, 4.5, 2.7)},
        {"name": c + "knuckle", "geometry": mesh(c + "knuckle"),
         "frames": [
             {"name": "hub", "T": r(sub(h["wheel"], knuckle))},
             {"name": "lower_ball_joint",
              "T": r(sub(h["lower_ball_joint"], knuckle))},
             {"name": "upper_ball_joint",
              "T": r(sub(h["upper_ball_joint"], knuckle))},
             {"name": "tie_rod", "T": r(sub(h["tie_rod_outer"], knuckle))},
             {"name": "halfshaft",
              "T": r(sub(h["halfshaft_outer"], knuckle))}],
         "center_of_mass": [0.0, s * 0.05, 0.0],
         "inertia": inertia(22.0, 0.35, 0.3, 0.2)},
        {"name": c + "lower_arm", "geometry": mesh(c + "lower_arm"),
         "frames": [
             {"name": "ball_joint",
              "T": r(sub(h["lower_ball_joint"], lower_arm))},
             {"name": "coil_over",
              "T": r(sub(h["coil_over_bottom"], lower_arm))}],
         "center_of_mass": [0.0, s * 0.25, 0.02],
         "inertia": inertia(14.0, 0.35, 0.25, 0.5)},
        {"name": c + "upper_arm", "geometry": mesh(c + "upper_arm"),
         "frames": [{"name": "ball_joint",
                     "T": r(sub(h["upper_ball_joint"], upper_arm))}],
         "center_of_mass": [0.0, s * 0.15, 0.0],
         "inertia": inertia(6.0, 0.06, 0.06, 0.1)},
        {"name": c + "halfshaft", "geometry": mesh(c + "halfshaft"),
         "center_of_mass": r([0.5 * x for x in
                              sub(h["halfshaft_outer"],
                                  h["halfshaft_inner"])]),
         "inertia": inertia(9.0, 0.1, 0.01, 0.1)},
        {"name": c + "tie_rod", "geometry": [],
         "frames": [{"name": "outer",
                     "T": r(sub(h["tie_rod_outer"], h["tie_rod_inner"]))}],
         "center_of_mass": r([0.5 * x for x in
                              sub(h["tie_rod_outer"], h["tie_rod_inner"])]),
         "inertia": inertia(1.5, 0.03, 0.0005, 0.03)}]
    chassis_frames += [
        {"name": c + "lower_arm", "T": r(lower_arm)},
        {"name": c + "upper_arm", "T": r(upper_arm)},
        {"name": c + "coil_over", "T": r(h["coil_over_top"])},
        {"name": c + "halfshaft", "T": r(h["halfshaft_inner"])}]
    rack_frames.append({"name": c + "tie_rod",
                        "T": r(sub(h["tie_rod_inner"], rack_origin))})

    joint(c + "lower_arm_pivot", "revolute", [["chassis"], c + "lower_arm"],
          [[c + "lower_arm"], "origin"], axis=[1, 0, 0])
    joint(c + "upper_arm_pivot", "revolute", [["chassis"], c + "upper_arm"],
          [[c + "upper_arm"], "origin"],
          axis=r(unit(h["upper_arm_axis"]), 9))
    joint(c + "lower_ball_joint", "spherical",
          [[c + "lower_arm"], "ball_joint"],
          [[c + "knuckle"], "lower_ball_joint"])
    joint(c + "upper_ball_joint", "spherical",
          [[c + "upper_arm"], "ball_joint"],
          [[c + "knuckle"], "upper_ball_joint"])
    joint(c + "hub", "revolute", [[c + "knuckle"], "hub"],
          [[c + "wheel"], "origin"], axis=[0, 1, 0],
          initial_displacement=0.0,
          initial_velocity=round(-speed / radius, 4),
          damping_coefficients=[0.0, 0.5])
    coil_over(c + "coil_over", [["chassis"], c + "coil_over"],
              [[c + "lower_arm"], "coil_over"], h["coil_over_top"],
              h["coil_over_bottom"], front, c + "coil_over", None,
              (6.0, 4.0))
    slide = telescope(c + "halfshaft", [["chassis"], c + "halfshaft"],
                      h["halfshaft_inner"], h["halfshaft_outer"],
                      c + "halfshaft", 1.0)
    joint(c + "halfshaft_outer", "spherical", [[slide], "origin"],
          [[c + "knuckle"], "halfshaft"])
    # tie rod: ball joint at the rack, universal joint at the knuckle
    rod = sub(h["tie_rod_outer"], h["tie_rod_inner"])
    joint(c + "tie_rod_inner", "spherical",
          [["steering_rack"], c + "tie_rod"], [[c + "tie_rod"], "origin"])
    universal(c + "tie_rod_outer", [[c + "tie_rod"], "outer"],
              [[c + "knuckle"], "tie_rod"], [0.0, 0.0, 1.0], rod)

    force_modules.append({
        "type": "tire", "name": c + "tire", "tire_type": "fiala",
        "wheel_body": c + "wheel",
        "fiala": {"unloaded_radius": radius, "width": 0.32,
                  "vertical_stiffness": 300000.0,
                  "vertical_damping": 3000.0}})

    # rear
    c = side + "_rear_"
    h = {k: m(v) for k, v in rear.items()}
    bodies += [
        {"name": c + "wheel", "geometry": mesh(c + "wheel"),
         "center_of_mass": [0.0, 0.0, 0.0],
         "inertia": inertia(50.0, 2.7, 4.5, 2.7)}]
    axle_frames += [
        {"name": c + "hub", "T": r(sub(h["wheel"], axle_origin))},
        {"name": c + "lower_link",
         "T": r(sub(h["lower_link_rear"], axle_origin))},
        {"name": c + "upper_link",
         "T": r(sub(h["upper_link_rear"], axle_origin))},
        {"name": c + "coil_over",
         "T": r(sub(h["coil_over_bottom"], axle_origin))}]
    chassis_frames += [
        {"name": c + "lower_link", "T": r(h["lower_link_front"])},
        {"name": c + "upper_link", "T": r(h["upper_link_front"])},
        {"name": c + "coil_over", "T": r(h["coil_over_top"])}]
    joint(c + "hub", "revolute", [["rear_axle"], c + "hub"],
          [[c + "wheel"], "origin"], axis=[0, 1, 0],
          initial_displacement=0.0,
          initial_velocity=round(-speed / radius, 4),
          damping_coefficients=[0.0, 0.5])
    for link, mass in (("lower_link", 7.0), ("upper_link", 6.0)):
        front_point, rear_point = h[link + "_front"], h[link + "_rear"]
        along = sub(rear_point, front_point)
        bodies.append({
            "name": c + link, "geometry": mesh(c + link),
            "frames": [{"name": "rear", "T": r(along)}],
            "center_of_mass": r([0.5 * x for x in along]),
            "inertia": inertia(mass, 0.01, 0.45, 0.45)})
        if link == "upper_link":
            # bushings: 2 MN/m along the link, about 12 Hz in pitch
            length = norm(along)
            slide = telescope(
                c + link, [["chassis"], c + link], front_point, rear_point,
                c + link, 0.5,
                stiffness_coefficients=[round(-2.0e6 * length, 1), 2.0e6],
                damping_coefficients=[0.0, 3000.0])
            joint(c + link + "_rear", "spherical", [[slide], "origin"],
                  [["rear_axle"], c + link])
        else:
            universal(c + link + "_front", [["chassis"], c + link],
                      [[c + link], "origin"], [0.0, 1.0, 0.0], along)
            joint(c + link + "_rear", "spherical", [[c + link], "rear"],
                  [["rear_axle"], c + link])
    coil_over(c + "coil_over", [["chassis"], c + "coil_over"],
              [["rear_axle"], c + "coil_over"], h["coil_over_top"],
              h["coil_over_bottom"], rear, c + "coil_upper",
              c + "coil_lower", (5.0, 4.0))
    force_modules.append({
        "type": "tire", "name": c + "tire", "tire_type": "fiala",
        "wheel_body": c + "wheel",
        "fiala": {"unloaded_radius": radius, "width": 0.32,
                  "vertical_stiffness": 300000.0,
                  "vertical_damping": 3000.0}})
    controllers.append({
        "type": "pid", "name": c + "drive", "kp": -100.0, "ki": -400.0,
        "kd": 0.0, "target": speed,
        "comment": "rear wheel drive: wheel torque to hold the speed",
        "sensor": "chassis_Tx_velocity", "joint": c + "hub"})

# rear axle, with the Panhard rod from the frame on the right to the axle on
# the left, and the prop shaft from the transfer case to the differential
axle_frames += [
    {"name": "panhard_rod", "T": r(sub(panhard_axle, axle_origin))},
    {"name": "prop_shaft", "T": r(sub(prop_rear, axle_origin))}]
chassis_frames += [{"name": "rear_axle", "T": r(axle_origin)},
                   {"name": "panhard_rod", "T": r(panhard_chassis)},
                   {"name": "prop_shaft", "T": r(prop_front)}]
bodies.append({"name": "rear_axle", "geometry": mesh("rear_axle"),
               "frames": axle_frames, "center_of_mass": [0.03, 0.0, 0.0],
               "comment": "estimated: housing, differential, shafts, brakes",
               "inertia": inertia(190.0, 55.0, 10.0, 55.0)})
rod = sub(panhard_axle, panhard_chassis)
bodies.append({"name": "panhard_rod", "geometry": mesh("panhard_rod"),
               "frames": [{"name": "axle", "T": r(rod)}],
               "center_of_mass": r([0.5 * x for x in rod]),
               "inertia": inertia(5.0, 0.0005, 0.3, 0.3)})
universal("panhard_rod_frame", [["chassis"], "panhard_rod"],
          [["panhard_rod"], "origin"], [1.0, 0.0, 0.0], rod)
joint("panhard_rod_axle", "spherical", [["panhard_rod"], "axle"],
      [["rear_axle"], "panhard_rod"])
shaft = sub(prop_rear, prop_front)
bodies.append({"name": "prop_shaft", "geometry": mesh("prop_shaft"),
               "center_of_mass": r([0.5 * x for x in shaft]),
               "inertia": inertia(15.0, 0.01, 3.6, 3.6)})
slide = telescope("prop_shaft", [["chassis"], "prop_shaft"], prop_front,
                  prop_rear, "prop_shaft", 1.0)
joint("prop_shaft_rear", "spherical", [[slide], "origin"],
      [["rear_axle"], "prop_shaft"])

bodies.insert(0, {
    "name": "chassis", "geometry": mesh("chassis"),
    "frames": chassis_frames, "center_of_mass": [0.42, 0.0, -0.85],
    "comment": "estimated: frame, body, engine and transmission",
    "inertia": inertia(2150.0, 1100.0, 5000.0, 5300.0)})

model = {
    "model_parameters": {
        "version": "0.2",
        "units": {"angle": "radians", "length": "meters",
                  "mass": "kilograms", "time": "seconds"},
        "gravity": [0.0, 0.0, 9.81],
        "include_directories": ["."],
        "solvers": [{"type": "rnca", "infinity_norm_multiplier": 2.0}],
        "integrators": [{"type": "ode113", "time_span": 10.0,
                         "time_step": 0.02, "relative_error": 0.0001,
                         "absolute_error": 0.0001, "omega_n": 50.0}]},
    "frames": [
        {"name": "overhead_1", "T": [8.0, -8.0, -8.0]},
        {"name": "overhead_2", "T": [-8.0, -8.0, -8.0]}],
    "lights": [
        {"name": "overhead_1", "type": "point",
         "parent": ["fixed", "overhead_1"], "Ka": [0.4, 0.4, 0.4],
         "Kd": [0.6, 0.6, 0.6], "Ks": [0.2, 0.2, 0.2]},
        {"name": "overhead_2", "type": "point",
         "parent": ["fixed", "overhead_2"], "Ka": [0.6, 0.6, 0.6],
         "Kd": [0.6, 0.6, 0.6], "Ks": [0.2, 0.2, 0.2]}],
    "assemblies": [{
        "name": "raptor", "active": True,
        "force_modules": force_modules,
        "sensors": [{"type": "frame", "name": "chassis_Tx_velocity",
                     "body_frame_pair": ["chassis", "origin"],
                     "quantity": "velocity", "coordinate_system": "local",
                     "dof": "Tx"}],
        "controllers": controllers, "bodies": bodies, "joints": joints}]}
with open(out, "w") as f:
    json.dump(model, f, indent=2)
print("bodies", len(bodies), "joints", len(joints))
