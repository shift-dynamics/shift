#!/usr/bin/env python3
"""The fastest run of the Polo R WRC through the forest stage found by a
search over schedules of its inputs: a genetic algorithm (--method ga, the
default), an evolution strategy (--method cma, CMA-ES from the cma
package) or model predictive path integral control (--method mppi, see
MPPI_SIGMA).

The steering rack, the throttle, the brake pedal, the handbrake and the gear
are schedules of time (knots every STEER_STEP and PEDAL_STEP s), in place of
the driver controller of make_polo_r_wrc_forest.py. The handbrake is the
Polo's (see ../polo_r_wrc/generate_polo_r_wrc.py): it locks the rear wheels
and opens the drive to the rear axle as it is pulled. Steering the whole
stage by a schedule is so sensitive to its start that the schedules are optimized over
a receding horizon: a window of WINDOW s at a time, the windows STRIDE s
apart, so that each overlaps the next by WINDOW - STRIDE s. The knots before
a window are kept, those in it are optimized, and each candidate is
simulated by shift-cli from the start to the end of its window; the window
reaches far enough ahead to take in the braking for the next bend, so that a
candidate that comes to it too fast scores badly by its own run. Only the
first STRIDE s of each window is settled; the rest starts the next window.
The first schedules are the inputs the driver controller gave (the log of
polo_r_wrc_forest.json).

In each window the genetic algorithm keeps the best ELITE schedules and
breeds the rest by tournaments, a blend of two parents and a mutation of one
stretch of the window (MUTATION_SPAN s), so that a change for the better in
one bend is not lost among changes in the others. CMA-ES instead samples all
the knots in the window about the best schedules, adapting the spread and
the correlation of the knots from the ranks of the runs. Either adds to a
share of its candidates a move, at a time in the window: braking for a
bend (the throttle lifted and the brake pressed once, firmly), or one that
starts a slide: a Scandinavian flick (the brakes tapped while the wheels are steered away
from the bend, then into it) or a handbrake turn (the throttle lifted and
the handbrake pulled while the wheels are steered into the bend), or a
release of the brake over a stretch.

A candidate is scored by how far along the road it is at the end of its
run, or, once it crosses the finish line, FINISH_BEFORE_END m before the end
of the road, by how early, less penalties over its window that keep its
driving like a driver's: for
inputs that go back and forth (corrections of the steering, alternating
pedals) through a turn, but not for a quick steer or a tap of the brakes to
start a slide, for braking while cornering (a driver brakes in a straight
line before the bend and accelerates out of it), for curving more than the
road (a swerve on a straight), for time off the gravel, and for ending
the run, RUN_ON s
after the finish line, on the straight past the end of the road away from
its middle, at an angle to it or turning (see OSCILLATION). Mutations change
the steering and the throttle by smooth bumps; the brake comes on only by
the moves that start a slide (and while no candidate survives, a lift and a
braking), and may be pressed with the throttle. It fails if
the car goes more than OFF_COURSE m from the middle of the road, comes
within reach of a tree, a sapling or a rock (their positions
from shift-cli --scenery, their radii measured from their meshes), rolls
over or turns back; a run that fails ranks below every run that does not,
by how far it got. Leaving the gravel a little, sliding and jumping are
allowed.

The driver controller stays in the model with its outputs unconnected: it
reports the station of the car along the road and its distance from the
middle.

Writes polo_r_wrc_forest_schedule.json next to this script (or --output):
the stage with the best schedules. The runs go in a work directory (--work),
with the state after each window, from which the search resumes.

    python3 optimize_polo_r_wrc_forest.py --work <dir> [--workers 30]
        [--method ga|cma|mppi] [--output <file>] [--seed-log <csv>]
"""

import argparse
import concurrent.futures
import copy
import json
import math
import os
import shutil
import subprocess
import sys
import time

import cma
import numpy as np
from scipy.ndimage import maximum_filter1d
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_polo_r_wrc_forest  # noqa: E402

ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
SHIFT_CLI = os.path.join(ROOT, "build-release", "shift", "main", "shift-cli")

# the schedules: knots of the steering rack (m) every STEER_STEP s, of the
# throttle, brake pedal and handbrake (0 to 1) and the gear (1 to 6) every
# PEDAL_STEP s, short enough for a tap of the brakes
STEER_STEP = 0.25
PEDAL_STEP = 0.25
RACK_LIMIT = 0.5 * make_polo_r_wrc_forest.RACK_PER_RADIAN
GEARS = (1, 6)
# the windows optimized in turn
WINDOW = 8.0
STRIDE = 3.0
# the search in each window, the knots in units of SCALE: for the genetic
# algorithm the spread of a mutation at first (falling by SIGMA_DECAY a
# generation), for CMA-ES the knots about the best schedules, sampled with
# the step SIGMA0 at first
POPULATION = 25
GENERATIONS = 12
MAX_GENERATIONS = 36
SCALE = {"steer": 0.006, "throttle": 0.2, "brake": 0.2, "handbrake": 0.3,
         "gear": 0.5}
ELITE = 2
TOURNAMENT = 3
GEAR_MUTATION = 0.08
MUTATION_SPAN = (1.0, 3.0)
BUMP = 2.0
SIGMA_DECAY = 0.85
SIGMA0 = 0.2
CMA_MOVES = 0.35
# MPPI (model predictive path integral control, Williams, Aldrich and
# Theodorou, Model Predictive Path Integral Control: From Theory to Parallel
# Computation, Journal of Guidance, Control, and Dynamics 40, 2017): the
# spread of the samples about the plan in units of SCALE, smoothed over
# MPPI_SMOOTH s, and the temperature of the weights (m)
MPPI_SIGMA = 0.5
MPPI_SMOOTH = 0.5
MPPI_TEMPERATURE = 3.0
# the moves, as a share of the candidates (MOVES; of the children of the
# genetic algorithm), a quarter each flicks, handbrake turns, braking for a
# bend and releasing the brake: for a flick, a tap of the brakes
# and a short steer away from the bend (FLICK_AWAY s), then the steering into
# it (FLICK_IN s), of a fraction of the rack's travel; for a handbrake turn,
# the handbrake pulled for HANDBRAKE_PULL s with the throttle lifted, the
# wheels steered further into the bend; for braking, the throttle lifted and
# the brake pressed BRAKE of the way for BRAKE_LENGTH s, the one press of
# the brake before a bend that the noise of the mutations never makes; for
# a release, the brake eased to RELEASE of itself over RELEASE_LENGTH s, so
# that braking through a bend, which the noise only eases slowly, can go
MOVES = 0.5
RELEASE = (0.0, 0.3)
RELEASE_LENGTH = (0.5, 2.0)
FLICK_AWAY = 0.5
FLICK_IN = 0.75
FLICK_AMPLITUDE = (0.2, 0.8)
FLICK_BRAKE = (0.3, 0.9)
HANDBRAKE_PULL = (0.25, 0.75)
HANDBRAKE = (0.5, 1.0)
HANDBRAKE_STEER = (0.1, 0.5)
BRAKE_LENGTH = (0.75, 2.0)
BRAKE = (0.3, 1.0)
# the pacenotes: the bends of the road tighter than BEND_RADIUS m (its
# curvature averaged over BEND_SMOOTH m), and the braking for one started
# BRAKE_BEFORE m before its entry, at the time the plan gets there; the
# braking of a move is for a bend ahead in the window, once the plan of the
# window has run
BEND_RADIUS = 45.0
BEND_SMOOTH = 10.0
BRAKE_BEFORE = (10.0, 60.0)
# a flick or a handbrake turn starts this far (m) before the entry of a bend
TURN_IN_BEFORE = (0.0, 30.0)
# while no candidate of a window survives, a share of the children of the
# genetic algorithm (RESCUE) lift and brake for a while at a time in the
# window, and the spread of the mutations does not fall
RESCUE = 0.3
RESCUE_LENGTH = (0.5, 1.5)
RESCUE_BRAKE = (0.4, 1.0)
# the log of the runs (s)
LOG_STEP = 0.05
TIME_LIMIT = 75.0
# the car's outline about the chassis origin, at the middle of the
# wheelbase, from the chassis mesh (m): x from rear to front, y half width
OUTLINE = (-1.85, 2.00, 0.92)
# the radius of each scenery item (m, at scale 1), measured from its mesh
# 0.2 to 1.2 m above the ground: the trunks of the trees with their roots,
# the spread of the saplings' stems, half the width of the rocks
RADII = {
    "fir_a": 0.73, "fir_b": 0.69, "fir_c": 0.67,
    "pine_a": 0.73, "pine_b": 0.69, "pine_c": 0.67,
    "fir_sapling_a": 0.15, "fir_sapling_b": 0.15, "fir_sapling_c": 0.15,
    "pine_sapling_a": 0.15, "pine_sapling_b": 0.15, "pine_sapling_c": 0.15,
    "boulder": 0.92, "stump": 0.71, "dead_trunk": 1.53,
    "mossy_rock_a": 1.69, "mossy_rock_b": 1.63, "mossy_rock_c": 1.06,
    "mossy_rock_d": 1.07, "mossy_rock_e": 1.50, "mossy_rock_f": 1.38,
}
OBSTACLES = ("trees", "saplings", "rocks")
# the penalties of a run, in meters of road, over the knots and the log of
# its window: for an input that goes back and forth (the steering, in units
# of RACK_LIMIT, the throttle and the brake), the swing between two of its
# reversals less than OSCILLATION_TIME s apart, so that a quick steer or a
# tap of the brakes to start a slide (one reversal) costs nothing, and
# corrections or alternating pedals through a turn do; for leaving the gravel, the square
# of the distance beyond its edge (GRAVEL m from the middle) over time; and,
# for a run that crosses the finish line, the square of the distance from
# the middle of the road, of the angle of the car to it and of its rate of
# yaw at the end of the run, so that the simulation ends with the car going
# straight down the middle of the road
OSCILLATION_TIME = 1.0       # s
OSCILLATION = {"steer": 30.0, "throttle": 24.0, "brake": 24.0}  # m per unit
GRAVEL = 3.4                 # m
OFF_ROAD_PENALTY = 4.0       # m per m^2 s
END_LATERAL = 20.0           # m per m^2
END_HEADING = 2000.0         # m per rad^2
# braking while cornering: the brake pedal times the lateral acceleration of
# the car beyond BRAKE_FREE_LATERAL, over g, over time; braking in a straight
# line before a bend costs nothing, nor trailing the brake lightly (up to
# TRAIL_BRAKE) into the turn while the car's lateral acceleration builds;
# a tap of the brakes at the turn in to start a slide costs little, and
# braking through the bend much
BRAKE_IN_TURN = 200.0        # m per s
BRAKE_FREE_LATERAL = 2.0     # m/s^2
TRAIL_BRAKE = 0.2
# an application of the brake this short (s) or shorter, a tap that starts a
# slide, does not count as braking in the turn
BRAKE_TAP = 0.5
# slides: in each bend tighter than BEND_RADIUS, for the car not
# oversteering (its heading turned into the bend from its direction of
# travel) by SLIDE_ANGLE at the most, this much a radian short of it; a
# driver on gravel slides the car through the tight bends
SLIDE = 60.0                 # m per rad
SLIDE_ANGLE = math.radians(15.0)
SLIDE_SPEED = 8.0            # m/s: slower than this, the slip is not scored
SLIDE_BEFORE = 15.0          # m before the entry of a bend that count
# swerving: the square of the curvature of the path of the car beyond the
# largest of the road within CURVE_NEAR of it, and CURVE_FREE, over the
# distance; taking a straighter line than the road costs nothing, nor
# turning in early or late
CURVE_EXCESS = 2.0e4         # m per m^-1
CURVE_FREE = 0.004           # 1/m
CURVE_NEAR = 10.0            # m
CURVE_SPEED = 3.0            # m/s: slower than this, the path is not scored
CURVE_SMOOTH = 0.5           # s, over which the direction of travel is smoothed
END_YAW_RATE = 300.0         # m per (rad/s)^2, over the last END_YAW_TIME s
END_YAW_TIME = 0.5           # s
# after the finish line the car runs on down the straight: the handbrake, and
# the brake beyond RUN_ON_FREE_BRAKE, cost this much a second, so that it is
# not set up for a turn there is no need of
RUN_ON_BRAKE = 400.0         # m per s
RUN_ON_FREE_BRAKE = 0.3
# the flying finish: a line across the road this far (m) before its end,
# after which the car may slow
FINISH_BEFORE_END = 30.0
# past the end of the road of make_forest.py its gravel runs on straight,
# along the road's last direction, to the edge of the terrain, about 78 m:
# this much of it is added to the path of the driver, which reports where
# the car is along it; the run goes on RUN_ON s after the finish line,
# along the straight, and is scored where it ends
RUN_OUT = 70.0               # m
RUN_ON = 3.0                 # s
ROLL_LIMIT = 0.8             # rad of roll or pitch: rolled over
OFF_COURSE = 6.0             # m from the middle of the road: left it
FAILED = -1.0e4              # the score of a failed run, plus its station
FINISH_SPEED = 25.0          # m/s: a second finished early is worth this

COLUMNS = {
    "time": "time",
    "x": "polo_r_wrc__chassis_x__q_0", "y": "polo_r_wrc__chassis_y__q_0",
    "roll": "polo_r_wrc__chassis_X__q_0",
    "pitch": "polo_r_wrc__chassis_Y__q_0",
    "yaw": "polo_r_wrc__chassis_Z__q_0",
    "station": "polo_r_wrc__driver__station",
    "lateral": "polo_r_wrc__driver__lateral_error",
    "speed": "polo_r_wrc__driver__vehicle_speed",
}


def absolute(value):
    """the paths of a model, relative to this directory, made absolute so
    that the model can be written anywhere"""
    if isinstance(value, dict):
        return {k: absolute(v) for k, v in value.items()}
    if isinstance(value, list):
        return [absolute(v) for v in value]
    if isinstance(value, str) and value.startswith(("../", "./")):
        return os.path.normpath(os.path.join(HERE, value)) + \
            ("/" if value.endswith("/") else "")
    return value


class Schedules:
    """the knots of the five schedules"""

    NAMES = ("steer", "throttle", "brake", "handbrake", "gear")

    def __init__(self, steer, throttle, brake, handbrake, gear):
        self.steer = np.asarray(steer, float)
        self.throttle = np.asarray(throttle, float)
        self.brake = np.asarray(brake, float)
        self.handbrake = np.asarray(handbrake, float)
        self.gear = np.asarray(gear, int)

    def copy(self):
        return Schedules(*(getattr(self, n).copy() for n in self.NAMES))

    def to_json(self):
        return {n: getattr(self, n).tolist() for n in self.NAMES}

    @staticmethod
    def from_json(data):
        handbrake = data.get("handbrake", np.zeros(len(data["brake"])))
        return Schedules(data["steer"], data["throttle"], data["brake"],
                         handbrake, data["gear"])

    def clip(self):
        np.clip(self.steer, -RACK_LIMIT, RACK_LIMIT, out=self.steer)
        np.clip(self.throttle, 0.0, 1.0, out=self.throttle)
        np.clip(self.brake, 0.0, 1.0, out=self.brake)
        np.clip(self.handbrake, 0.0, 1.0, out=self.handbrake)
        np.clip(self.gear, GEARS[0], GEARS[1], out=self.gear)


def knot_times(step):
    return np.arange(0.0, TIME_LIMIT + 1e-9, step)


def points(times, values):
    return [float(v) for pair in zip(times, values) for v in pair]


def base_model():
    """the stage of make_polo_r_wrc_forest.py with the driver observing:
    its maps replaced by the five schedules (filled in by model())"""
    model, _ = make_polo_r_wrc_forest.build_model()
    model = absolute(model)
    params = model["model_parameters"]
    params["include_directories"] = [HERE, os.path.join(ROOT, "examples",
                                                        "polo_r_wrc")]
    params["integrators"][0]["time_step"] = LOG_STEP
    assembly = model["assemblies"][0]
    keep = []
    for c in assembly["controllers"]:
        if c["type"] == "driver":
            path = np.array(c["path"], float).reshape(-1, 2)
            tangent = path[-1] - path[-2]
            tangent /= np.linalg.norm(tangent)
            step = float(np.linalg.norm(path[-1] - path[-2]))
            along = np.arange(1, round(RUN_OUT / step) + 1) * step
            more = path[-1] + np.outer(along, tangent)
            c["path"] = [round(float(v), 3) for v in
                         np.vstack([path, more]).ravel()]
            keep.append(c)
        elif c["name"] == "brake_lights":
            c["component"] = "brake_pedal"
            c["output"] = "value"
            keep.append(c)
        elif c["name"] in ("clutch_pedal", "steering", "handbrake",
                           "rear_drive"):
            keep.append(c)
    brakes = ["force_modules/drive/brakes/" + side + "_" + end + "/pressure"
              for side in ("left", "right") for end in ("front", "rear")]
    schedules = [
        {"type": "schedule", "name": "steering_wheel",
         "interpolation": "linear",
         "targets": ["controllers/steering/target"]},
        {"type": "schedule", "name": "throttle", "interpolation": "linear",
         "targets": ["force_modules/drive/engine/throttle"]},
        {"type": "schedule", "name": "brake_pedal",
         "interpolation": "linear", "targets": brakes},
        {"type": "schedule", "name": "gears", "interpolation": "step",
         "targets": ["force_modules/drive/transmission/gear"]},
    ]
    assembly["controllers"] = schedules + keep
    return model


def model(base, schedules, time_span, logfile):
    m = copy.deepcopy(base)
    m["model_parameters"]["integrators"][0]["time_span"] = time_span
    m["model_parameters"]["loggers"][0]["csv_logfile"] = logfile
    ts, tp = knot_times(STEER_STEP), knot_times(PEDAL_STEP)
    values = {"steering_wheel": (ts, schedules.steer),
              "throttle": (tp, schedules.throttle),
              "brake_pedal": (tp, schedules.brake),
              "handbrake": (tp, schedules.handbrake),
              "gears": (tp, schedules.gear)}
    for c in m["assemblies"][0]["controllers"]:
        if c["name"] in values:
            c["points"] = points(*values[c["name"]])
    return m


def read_log(path):
    with open(path) as f:
        header = f.readline().strip().split(",")
    index = {k: header.index(v) for k, v in COLUMNS.items()}
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2,
                      usecols=sorted(index.values()))
    order = sorted(index.values())
    return {k: data[:, order.index(i)] for k, i in index.items()}


class Stage:
    """the obstacles near the road, and the length of the road"""

    def __init__(self, base, work):
        path = os.path.join(work, "scenery.csv")
        if not os.path.exists(path):
            stage = os.path.join(work, "stage.json")
            with open(stage, "w") as f:
                json.dump(model(base, seed_schedules_flat(), 1.0,
                                "stage.csv"), f)
            subprocess.run([SHIFT_CLI, "-v", "warning", "--scenery", path,
                            stage], cwd=work, check=True,
                           stdout=subprocess.DEVNULL)
        xy, radius = [], []
        with open(path) as f:
            f.readline()
            for line in f:
                name, _, mesh, x, y, _, scale = line.strip().split(",")
                if name not in OBSTACLES:
                    continue
                key = os.path.splitext(os.path.basename(mesh))[0]
                xy.append((float(x), float(y)))
                radius.append(RADII[key] * float(scale))
        driver = next(c for c in base["assemblies"][0]["controllers"]
                      if c["type"] == "driver")
        road = np.array(driver["path"]).reshape(-1, 2)
        self.road_s = np.concatenate([[0.0], np.cumsum(
            np.linalg.norm(np.diff(road, axis=0), axis=1))])
        tangent = np.gradient(road, self.road_s, axis=0)
        self.road_heading = np.unwrap(np.arctan2(tangent[:, 1],
                                                 tangent[:, 0]))
        self.length = float(self.road_s[-1])
        self.finish = self.length - RUN_OUT - FINISH_BEFORE_END
        xy, radius = np.array(xy), np.array(radius)
        near = cKDTree(road).query(xy)[0] < 25.0
        self.xy, self.radius = xy[near], radius[near]
        self.tree = cKDTree(self.xy)
        curvature = np.gradient(self.road_heading, self.road_s)
        n = max(int(round(BEND_SMOOTH / np.mean(np.diff(self.road_s)))), 1)
        curvature = np.convolve(curvature, np.ones(n) / n, "same")
        near = max(int(round(CURVE_NEAR / np.mean(np.diff(self.road_s)))),
                   1)
        self.curvature_near = maximum_filter1d(np.abs(curvature),
                                               2 * near + 1)
        tight = np.abs(curvature) > 1.0 / BEND_RADIUS
        self.bends = self.road_s[tight & ~np.concatenate([[False],
                                                          tight[:-1]])]
        # each bend: from its entry to its exit, and the side it turns to
        exits = self.road_s[tight & ~np.concatenate([tight[1:], [False]])]
        self.bend_spans = [
            (float(a), float(b), float(np.sign(np.mean(
                curvature[(self.road_s >= a) & (self.road_s <= b)]))))
            for a, b in zip(self.bends, exits)]
        self.reach = math.hypot(max(-OUTLINE[0], OUTLINE[1]), OUTLINE[2]) + \
            float(self.radius.max())

    def heading_error(self, station, yaw):
        """the angle of the car to the road at station (rad)"""
        d = yaw - np.interp(station, self.road_s, self.road_heading)
        return float(math.atan2(math.sin(d), math.cos(d)))

    def hit(self, log):
        """the first time the outline of the car reaches an obstacle, or
        None; the outline is turned by the yaw of the chassis about its
        roll and pitch, and the poses logged are interpolated to 1 cm/s
        steps"""
        t = log["time"]
        fine = np.arange(t[0], t[-1], 0.01)
        x = np.interp(fine, t, log["x"])
        y = np.interp(fine, t, log["y"])
        roll = np.interp(fine, t, log["roll"])
        pitch = np.interp(fine, t, log["pitch"])
        yaw = np.interp(fine, t, log["yaw"])
        # the chassis x and y axes in the fixed frame: R = Rx(roll) Ry(pitch)
        # Rz(yaw) applied to the axes, projected on the ground
        cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), \
            np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        ax = np.stack([cp * cy, sr * sp * cy + cr * sy], axis=1)
        ay = np.stack([-cp * sy, -sr * sp * sy + cr * cy], axis=1)
        for k, near in enumerate(self.tree.query_ball_point(
                np.stack([x, y], axis=1), self.reach)):
            for j in near:
                d = self.xy[j] - (x[k], y[k])
                u = np.dot(d, ax[k]) / np.dot(ax[k], ax[k])
                v = np.dot(d, ay[k]) / np.dot(ay[k], ay[k])
                du = max(OUTLINE[0] - u, 0.0, u - OUTLINE[1])
                dv = max(abs(v) - OUTLINE[2], 0.0)
                if math.hypot(du, dv) < self.radius[j]:
                    return float(fine[k])
        return None


def oscillation(times, values):
    """the swings between reversals of values less than OSCILLATION_TIME
    apart: the reversals are the knots at which the values, held steady or
    not, turn from rising to falling or back"""
    d = np.diff(values)
    moving = np.nonzero(np.abs(d) > 1e-6)[0]
    if len(moving) < 2:
        return 0.0
    sign = np.sign(d[moving])
    turns = moving[1:][sign[1:] != sign[:-1]]
    total = 0.0
    for a, b in zip(turns[:-1], turns[1:]):
        if times[b] - times[a] < OSCILLATION_TIME:
            total += abs(values[b] - values[a])
    return total


def penalty(stage, log, start, schedules, finished, t_finish=None):
    """the penalties of a run from start (see OSCILLATION), in m;
    finished: whether the log reaches the end of the run, RUN_ON s after
    the finish line, where the state of the car is scored (a window that
    ends sooner after the line would be scored for the car before it has
    settled, and gain by not finishing); t_finish: when it crossed the
    finish line, or None"""
    t = log["time"]
    m = t >= start
    off = np.maximum(np.abs(log["lateral"][m]) - GRAVEL, 0.0)
    total = OFF_ROAD_PENALTY * float(np.sum(off ** 2)) * float(t[1] - t[0])
    if finished:
        total += END_LATERAL * float(log["lateral"][-1]) ** 2
        total += END_HEADING * stage.heading_error(log["station"][-1],
                                                   log["yaw"][-1]) ** 2
        last = t >= t[-1] - END_YAW_TIME
        if np.count_nonzero(last) > 1:
            rate = np.gradient(np.unwrap(log["yaw"][last]), t[last])
            total += END_YAW_RATE * float(np.mean(rate ** 2))
    if t_finish is not None and schedules is not None:
        tp = knot_times(PEDAL_STEP)
        after = m & (t >= t_finish)
        brake = np.interp(t, tp, schedules.brake)
        handbrake = np.interp(t, tp, schedules.handbrake)
        total += RUN_ON_BRAKE * float(np.sum(
            (np.maximum(brake - RUN_ON_FREE_BRAKE, 0.0) + handbrake)[after])) \
            * float(t[1] - t[0])
    total += BRAKE_IN_TURN * brake_in_turn(log, m, schedules) + \
        CURVE_EXCESS * curve_excess(stage, log, m) + \
        SLIDE * slide_shortfall(stage, log, m)
    if schedules is not None:
        end = t[-1]
        ts, tp = knot_times(STEER_STEP), knot_times(PEDAL_STEP)
        # the knots of the run from start, with those up to
        # OSCILLATION_TIME before it
        ks = (ts >= start - OSCILLATION_TIME) & (ts <= end)
        kp = (tp >= start - OSCILLATION_TIME) & (tp <= end)
        total += OSCILLATION["steer"] * oscillation(
            ts[ks], schedules.steer[ks] / RACK_LIMIT)
        for name in ("throttle", "brake"):
            total += OSCILLATION[name] * oscillation(
                tp[kp], getattr(schedules, name)[kp])
    return total


def travel(log):
    """the speed of the car and the direction of its travel (rad), smoothed
    over CURVE_SMOOTH s, from its logged position"""
    t = log["time"]
    vx, vy = np.gradient(log["x"], t), np.gradient(log["y"], t)
    n = max(int(round(CURVE_SMOOTH / float(t[1] - t[0]))), 1)
    kernel = np.ones(n) / n
    vx = np.convolve(vx, kernel, "same")
    vy = np.convolve(vy, kernel, "same")
    return np.hypot(vx, vy), np.unwrap(np.arctan2(vy, vx))


def brake_in_turn(log, m, schedules):
    """the brake pedal, less TRAIL_BRAKE while the lateral acceleration of
    the car rises (the turn in), times the lateral acceleration beyond
    BRAKE_FREE_LATERAL, over g, over time (s), at the samples m of the
    log"""
    if schedules is None or np.count_nonzero(m) < 2:
        return 0.0
    t = log["time"]
    speed, direction = travel(log)
    lateral = np.abs(speed * np.gradient(direction, t))
    n = max(int(round(CURVE_SMOOTH / float(t[1] - t[0]))), 1)
    lateral = np.convolve(lateral, np.ones(n) / n, "same")
    brake = schedules.brake.copy()
    on = np.concatenate([[0], (brake > 0.02).astype(int), [0]])
    edges = np.diff(on)
    for a, b in zip(np.nonzero(edges == 1)[0], np.nonzero(edges == -1)[0]):
        if (b - a) * PEDAL_STEP <= BRAKE_TAP:
            brake[a:b] = 0.0
    brake = np.interp(t, knot_times(PEDAL_STEP), brake)
    turning_in = np.gradient(lateral, t) > 0.0
    brake = np.where(turning_in, np.maximum(brake - TRAIL_BRAKE, 0.0), brake)
    excess = np.maximum(lateral - BRAKE_FREE_LATERAL, 0.0) / 9.81
    return float(np.sum((brake * excess)[m])) * float(t[1] - t[0])


def curve_excess(stage, log, m):
    """the square of the curvature of the path of the car (1/m) beyond the
    largest of the road within CURVE_NEAR of it and CURVE_FREE, over the
    distance (m), at the samples m of the log, while it goes faster than
    CURVE_SPEED"""
    t = log["time"]
    if np.count_nonzero(m) < 2:
        return 0.0
    speed, direction = travel(log)
    ds = speed * float(t[1] - t[0])
    moving = m & (speed > CURVE_SPEED)
    curvature = np.zeros_like(t)
    curvature[moving] = np.gradient(direction, t)[moving] / speed[moving]
    road = np.interp(log["station"], stage.road_s, stage.curvature_near)
    excess = np.maximum(np.abs(curvature) - road - CURVE_FREE, 0.0)
    return float(np.sum((excess ** 2 * ds)[moving]))


def slide_shortfall(stage, log, m):
    """the sum over the bends the car goes through at the samples m of the
    log, faster than SLIDE_SPEED, of how far (rad) its largest oversteer
    in the bend falls short of SLIDE_ANGLE"""
    if np.count_nonzero(m) < 2:
        return 0.0
    speed, direction = travel(log)
    beta = np.angle(np.exp(1j * (log["yaw"] - direction)))
    station = log["station"]
    total = 0.0
    for a, b, side in stage.bend_spans:
        if station[-1] < b:
            continue
        inside = m & (station >= a - SLIDE_BEFORE) & (station <= b) & \
            (speed > SLIDE_SPEED)
        if not inside.any():
            continue
        oversteer = float(np.max(side * beta[inside]))
        total += max(SLIDE_ANGLE - oversteer, 0.0)
    return total


def score(stage, log, end, start=0.0, schedules=None):
    """the score of a run to time end (larger is better), less the
    penalties of it from start, why it failed, if it did, and when (or
    None); what happens later than RUN_ON s after the finish does not
    count"""
    t = log["time"]
    station = log["station"]

    def failed(k):
        return FAILED + float(station[k]), float(t[k])

    if t[-1] < end - 1.5 * LOG_STEP:
        return failed(-1)[0], "stopped at %.2f s" % t[-1], float(t[-1])
    t_finish = finish_time(stage, log)
    if t_finish is not None:
        kept = t <= t_finish + RUN_ON + 0.5 * LOG_STEP
        log = {k: v[kept] for k, v in log.items()}
        t, station = log["time"], log["station"]
    over = np.nonzero((np.abs(log["roll"]) > ROLL_LIMIT) |
                      (np.abs(log["pitch"]) > ROLL_LIMIT))[0]
    if len(over):
        value, when = failed(over[0])
        return value, "rolled over", when
    wide = np.nonzero(np.abs(log["lateral"]) > OFF_COURSE)[0]
    if len(wide):
        value, when = failed(wide[0])
        return value, "left the road", when
    back = np.nonzero(np.concatenate([[False], np.diff(station) < -2.0]) |
                      (log["speed"] < -1.0))[0]
    if len(back):
        value, when = failed(back[0])
        return value, "turned back", when
    hit = stage.hit(log)
    if hit is not None:
        return failed(np.searchsorted(t, hit))[0], "hit at %.2f s" % hit, \
            hit
    ended = t_finish is not None and \
        t[-1] >= t_finish + RUN_ON - 1.5 * LOG_STEP
    p = penalty(stage, log, start, schedules, ended, t_finish)
    if t_finish is not None:
        return stage.finish + FINISH_SPEED * (end - t_finish) - p, \
            "finished at %.3f s, penalty %.1f m" % (t_finish, p), None
    s = station[-1]
    return s - p, "station %.1f m, penalty %.1f m" % (s, p), None


def finish_time(stage, log):
    station = log["station"]
    finished = np.nonzero(station >= stage.finish)[0]
    if not len(finished):
        return None
    k = max(finished[0], 1)
    return float(np.interp(stage.finish, station[k - 1:k + 1],
                           log["time"][k - 1:k + 1]))


def run(base, schedules, end, directory):
    """simulate schedules to end in directory; the log, or None"""
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, "model.json")
    with open(path, "w") as f:
        json.dump(model(base, schedules, end, "run.csv"), f)
    try:
        subprocess.run([SHIFT_CLI, "-v", "warning", path], cwd=directory,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       timeout=60.0 + 3.0 * end)
        return read_log(os.path.join(directory, "run.csv"))
    except Exception:
        return None


def evaluate(base, stage, schedules, end, directory):
    """the score of schedules simulated to end, why, when it failed (or
    None), and the station at the knots of the pedals (nan after the log)"""
    log = run(base, schedules, end, directory)
    shutil.rmtree(directory, ignore_errors=True)
    if log is None or len(log["time"]) < 2:
        return FAILED, "no log", 0.0, None
    tp = knot_times(PEDAL_STEP)
    trace = np.interp(tp, log["time"], log["station"], right=np.nan)
    return score(stage, log, end, max(end - WINDOW, 0.0), schedules) + \
        (trace,)


def seed_schedules_flat():
    n_s, n_p = len(knot_times(STEER_STEP)), len(knot_times(PEDAL_STEP))
    return Schedules(np.zeros(n_s), np.zeros(n_p), np.zeros(n_p),
                     np.zeros(n_p), np.ones(n_p, int))


def realign(base, best, seed, reference, t_from, directory):
    """best with its knots from t_from on, which no window has searched yet,
    those of the seed schedules for the same stretch of road: shifted in time
    by when the run of the seed (its log, reference) was where best is at
    t_from, so that a run ahead of or behind the seed meets the seed's
    inputs for a bend where the bend is; and the shift (s)"""
    log = run(base, best, t_from, directory)
    if log is None or len(log["time"]) == 0:
        return best, 0.0
    station = float(log["station"][-1])
    if station >= float(np.max(reference["station"])) - 1.0:
        return best, 0.0
    shift = float(np.interp(station,
                            np.maximum.accumulate(reference["station"]),
                            reference["time"])) - t_from
    if abs(shift) < PEDAL_STEP / 2:
        return best, shift
    best = best.copy()
    for name, step in (("steer", STEER_STEP), ("throttle", PEDAL_STEP),
                       ("brake", PEDAL_STEP), ("handbrake", PEDAL_STEP),
                       ("gear", PEDAL_STEP)):
        t = knot_times(step)
        later = t >= t_from - 1e-9
        values = getattr(seed, name)
        if name == "gear":
            k = np.clip(np.rint((t[later] + shift) / step).astype(int), 0,
                        len(values) - 1)
            getattr(best, name)[later] = values[k]
        else:
            getattr(best, name)[later] = np.interp(t[later] + shift, t,
                                                   values)
    best.clip()
    return best, shift


def realign_report(base, best, seed, reference, start, work, log_file):
    """best realigned (see realign) past the windows searched before the
    one that starts at start"""
    t_from = start + WINDOW - STRIDE
    best, shift = realign(base, best, seed, reference, t_from,
                          os.path.join(work, "realign"))
    line = "realigned the seed from %.1f s by %+.2f s" % (t_from, shift)
    print(line, flush=True)
    log_file.write(line + "\n")
    return best


def seed_schedules(path=None):
    """the inputs the driver controller gave, from the log of
    polo_r_wrc_forest.json (simulated by shift-cli if it is missing), or
    from the log at path of another run of the stage (as of the best line of
    optimize_polo_r_wrc_forest_line.py)"""
    if path is None:
        path = os.path.join(HERE, "polo_r_wrc_forest.csv")
    if not os.path.exists(path):
        subprocess.run([SHIFT_CLI, "-v", "warning", "polo_r_wrc_forest.json"],
                       cwd=HERE, check=True, stdout=subprocess.DEVNULL)
    with open(path) as f:
        header = f.readline().strip().split(",")
    names = ["time", "polo_r_wrc__steering_wheel__value",
             "polo_r_wrc__throttle__value", "polo_r_wrc__brake_pedal__value",
             "polo_r_wrc__gears__value"]
    data = np.loadtxt(path, delimiter=",", skiprows=1,
                      usecols=[header.index(n) for n in names])
    t = data[:, 0]
    ts, tp = knot_times(STEER_STEP), knot_times(PEDAL_STEP)
    s = Schedules(np.interp(ts, t, data[:, 1]), np.interp(tp, t, data[:, 2]),
                  np.interp(tp, t, data[:, 3]), np.zeros(len(tp)),
                  np.rint(np.interp(tp, t, data[:, 4])).astype(int))
    s.clip()
    return s


def window_masks(start, end):
    ts, tp = knot_times(STEER_STEP), knot_times(PEDAL_STEP)
    return (ts >= start) & (ts < end), (tp >= start) & (tp < end)


class Window:
    """the knots of a window as a vector for CMA-ES: in units of SCALE about
    the schedules it starts from, the gear rounded"""

    def __init__(self, span, schedules):
        self.start = schedules
        ms, mp = window_masks(*span)
        self.masks = {"steer": ms, "throttle": mp, "brake": mp,
                      "handbrake": mp, "gear": mp}
        self.size = {n: int(m.sum()) for n, m in self.masks.items()}
        # the times to start braking for the bends ahead (see
        # pacenote_times), once the plan of the window has run
        self.brake_times = None
        self.turn_times = None

    def encode(self, schedules):
        return np.concatenate([
            (getattr(schedules, n)[m] - getattr(self.start, n)[m]) / SCALE[n]
            for n, m in self.masks.items()])

    def decode(self, x):
        s = self.start.copy()
        i = 0
        for n, m in self.masks.items():
            v = getattr(self.start, n)[m] + SCALE[n] * x[i:i + self.size[n]]
            getattr(s, n)[m] = np.rint(v) if n == "gear" else v
            i += self.size[n]
        s.clip()
        return s

    def integers(self):
        """the indices of the gears in the vector"""
        i = sum(self.size[n] for n in Schedules.NAMES[:-1])
        return list(range(i, i + self.size["gear"]))


def bend_side(child, around):
    """the side of the bend from the steering there (1 or -1), 0 if the
    wheels are straight"""
    return float(np.sign(np.sum(child.steer[around])))


def move_time(masks, rng, times):
    """a time to start a move: in one of the ranges of times (see
    pacenote_times), or, if there are none, at a time in the window"""
    if times:
        lo, hi = times[rng.integers(len(times))]
        return rng.uniform(lo, hi)
    ts = knot_times(STEER_STEP)[masks["steer"]]
    return rng.uniform(ts[0], ts[-1])


def flick(child, masks, rng, times=None):
    """add to child, at the turn in of a bend ahead (times, see
    pacenote_times) or a time in the window, the inputs that start a slide
    (a Scandinavian flick): the brakes tapped and the throttle lifted while
    the wheels are steered away from the bend, then steered into it"""
    ms, mp = masks["steer"], masks["brake"]
    ts, tp = knot_times(STEER_STEP), knot_times(PEDAL_STEP)
    t0 = move_time(masks, rng, times)
    around = ms & (ts >= t0) & (ts < t0 + FLICK_AWAY + FLICK_IN)
    toward = bend_side(child, around) or rng.choice([-1.0, 1.0])
    a = rng.uniform(*FLICK_AMPLITUDE) * RACK_LIMIT
    away = ms & (ts >= t0) & (ts < t0 + FLICK_AWAY)
    into = ms & (ts >= t0 + FLICK_AWAY) & (ts < t0 + FLICK_AWAY + FLICK_IN)
    child.steer[away] -= toward * a
    child.steer[into] += toward * a
    tap = mp & (tp >= t0) & (tp < t0 + FLICK_AWAY)
    child.brake[tap] = np.maximum(child.brake[tap],
                                  rng.uniform(*FLICK_BRAKE))
    child.throttle[tap] = 0.0


def handbrake_turn(child, masks, rng, times=None):
    """add to child, at the turn in of a bend ahead (times, see
    pacenote_times) or a time in the window, a handbrake turn: the
    throttle lifted and the handbrake pulled while the wheels are steered
    further into the bend"""
    ms, mp = masks["steer"], masks["handbrake"]
    ts, tp = knot_times(STEER_STEP), knot_times(PEDAL_STEP)
    t0 = move_time(masks, rng, times)
    pull = rng.uniform(*HANDBRAKE_PULL)
    into = ms & (ts >= t0) & (ts < t0 + pull)
    toward = bend_side(child, into) or rng.choice([-1.0, 1.0])
    child.steer[into] += toward * rng.uniform(*HANDBRAKE_STEER) * RACK_LIMIT
    held = mp & (tp >= t0) & (tp < t0 + pull)
    child.handbrake[held] = rng.uniform(*HANDBRAKE)
    child.throttle[held] = 0.0


def evaluate_all(base, stage, population, end, pool, work, counter):
    jobs = []
    for candidate in population:
        counter[0] += 1
        directory = os.path.join(work, "runs", "%06d" % counter[0])
        jobs.append(pool.submit(evaluate, base, stage, candidate, end,
                                directory))
    return [j.result() for j in jobs]


def report(log_file, start, generation, top, scores, counter, extra=""):
    failed = int(np.sum(scores <= FAILED / 2))
    line = ("window %5.1f-%5.1f s  generation %2d  best %8.2f (%s)  "
            "failed %2d/%d  runs %d%s" %
            (start, start + WINDOW, generation, top[0], top[1], failed,
             len(scores), counter[0], extra))
    print(line, flush=True)
    log_file.write(line + "\n")
    log_file.flush()


def lift_and_brake(child, masks, rng, length=RESCUE_LENGTH,
                   strength=RESCUE_BRAKE, t0=None):
    """add to child, at a time in the window (or at t0), a lift of the
    throttle and a braking: a way out of a window in which every candidate
    fails, or, with BRAKE_LENGTH and BRAKE, the braking for a bend"""
    mp = masks["brake"]
    tp = knot_times(PEDAL_STEP)
    if t0 is None:
        t0 = rng.uniform(tp[mp][0], tp[mp][-1])
    held = mp & (tp >= t0) & (tp < t0 + rng.uniform(*length))
    child.throttle[held] = 0.0
    child.brake[held] = np.maximum(child.brake[held], rng.uniform(*strength))


def pacenote_times(stage, trace, start, end, before=BRAKE_BEFORE):
    """the ranges of time (s) in which to start braking for the bends whose
    entries the plan (its station at the knots of the pedals) reaches in
    the window [start, end) or just after it: from before[1] to before[0] m
    before each entry (BRAKE_BEFORE; TURN_IN_BEFORE, for the moves that
    start a slide)"""
    if trace is None:
        return None
    tp = knot_times(PEDAL_STEP)
    ok = np.isfinite(trace)
    t, s = tp[ok], np.maximum.accumulate(trace[ok])
    s0, s1 = np.interp(start, t, s), np.interp(end, t, s)
    times = []
    for entry in stage.bends:
        if s0 < entry < s1 + before[0]:
            lo, hi = np.interp([entry - before[1], entry - before[0]], s, t)
            lo, hi = max(lo, start), min(hi, end - PEDAL_STEP)
            if hi > lo:
                times.append((float(lo), float(hi)))
    return times or None


def brake_for_bend(child, masks, rng, times=None):
    """add to child the braking for a bend: for one of the bends ahead
    (times, see pacenote_times), or, if there are none, at a time in the
    window"""
    t0 = None
    if times:
        lo, hi = times[rng.integers(len(times))]
        t0 = rng.uniform(lo, hi)
    lift_and_brake(child, masks, rng, BRAKE_LENGTH, BRAKE, t0)


def release_brake(child, masks, rng):
    """ease the brake of child to RELEASE of itself over RELEASE_LENGTH s,
    at a time in the window where it is on (or anywhere, if it is not)"""
    mp = masks["brake"]
    tp = knot_times(PEDAL_STEP)
    on = np.nonzero(mp & (child.brake > 0.02))[0]
    length = rng.uniform(*RELEASE_LENGTH)
    if len(on):
        t0 = tp[on[rng.integers(len(on))]] - rng.uniform(0.0, length)
    else:
        t0 = rng.uniform(tp[mp][0], tp[mp][-1])
    span = mp & (tp >= t0) & (tp < t0 + length)
    child.brake[span] *= rng.uniform(*RELEASE)


def move(child, window, rng, u, share):
    """one of the four moves, by u (uniform from 0 to 1) in quarters of
    share; none if u is past share"""
    if u < share / 4:
        flick(child, window.masks, rng, window.turn_times)
    elif u < share / 2:
        handbrake_turn(child, window.masks, rng, window.turn_times)
    elif u < 3 * share / 4:
        brake_for_bend(child, window.masks, rng, window.brake_times)
    elif u < share:
        release_brake(child, window.masks, rng)


def mutate(parent, window, scale, rng, rescue=False):
    """parent with one stretch of the window changed, and, for a share of
    the children, a move that starts a slide, or, while none survive
    (rescue), a lift and a braking"""
    child = parent.copy()
    ms, mp = window.masks["steer"], window.masks["throttle"]
    ts, tp = knot_times(STEER_STEP), knot_times(PEDAL_STEP)
    first, last = ts[ms][0], ts[ms][-1] + STEER_STEP
    length = min(rng.uniform(*MUTATION_SPAN), last - first)
    begin = rng.uniform(first, last - length)
    ms = ms & (ts >= begin) & (ts < begin + length)
    mp = mp & (tp >= begin) & (tp < begin + length)
    # a smooth bump over the stretch, of the steering and of the throttle:
    # noise knot by knot would make the steering jagged; the brake only
    # eased or pressed harder where it is already on (it comes on by the
    # moves below and by the rescue), since noise about 0, clipped at 0,
    # would tap the brakes at random
    def bump(times):
        return 0.5 * (1.0 - np.cos(2.0 * math.pi * (times - begin) / length))

    child.steer[ms] += rng.normal(0.0, BUMP * SCALE["steer"] * scale) * \
        bump(ts[ms])
    child.throttle[mp] += rng.normal(0.0, BUMP * SCALE["throttle"] *
                                     scale) * bump(tp[mp])
    child.brake[mp] *= 1.0 + rng.normal(0.0, SCALE["brake"] * scale) * \
        bump(tp[mp])
    # the handbrake only by the handbrake turns: noise about 0 would drag
    # it on everywhere; a mutation can only ease a pull already there
    if rng.random() < 0.5:
        child.handbrake[mp] *= rng.uniform(0.0, 1.0)
    flip = mp & (rng.random(len(mp)) < GEAR_MUTATION * scale)
    child.gear[flip] += rng.choice([-1, 1], flip.sum())
    u = rng.random()
    if rescue and rng.random() < RESCUE:
        lift_and_brake(child, window.masks, rng)
    else:
        move(child, window, rng, u, MOVES)
    child.clip()
    return child


def crossover(a, b, window, rng):
    """a blend of a and b in the window, per knot, of the steering and the
    throttle; each application of the brake and of the handbrake whole from
    one of them (a blend would spread a pull of one, weakened, over the
    knots); the gears of either"""
    child = a.copy()
    for name in ("steer", "throttle"):
        m = window.masks[name]
        u = rng.random(m.sum())
        va, vb = getattr(a, name)[m], getattr(b, name)[m]
        getattr(child, name)[m] = va + u * (vb - va)
    for name in ("brake", "handbrake"):
        m = window.masks[name]
        va, vb = getattr(a, name)[m], getattr(b, name)[m]
        on = (va > 0.0) | (vb > 0.0)
        # the stretches in which either is on, each from a or from b
        label = np.cumsum(np.concatenate([[on[0]], on[1:] & ~on[:-1]])) * on
        v = va.copy()
        for k in range(1, int(label.max()) + 1):
            if rng.random() < 0.5:
                v[label == k] = vb[label == k]
        getattr(child, name)[m] = v
    mp = window.masks["gear"]
    pick = rng.random(mp.sum()) < 0.5
    child.gear[mp] = np.where(pick, a.gear[mp], b.gear[mp])
    child.clip()
    return child


def behind_plan(plan, top):
    """whether the best (score, why, schedules, station at the knots) is
    behind the plan started from (score, why, when it failed, station at
    the knots) when the plan failed; never if the plan survived"""
    when = plan[2]
    if when is None:
        return False
    if top[0] <= FAILED / 2:
        return True
    # the score of a run that fails is FAILED plus its station then
    reached = plan[0] - FAILED
    return float(np.interp(when, knot_times(PEDAL_STEP), top[3])) < \
        reached - 1.0


def optimize_window_ga(base, stage, best, start, pool, work, rng, log_file,
                       counter):
    end = start + WINDOW
    window = Window((start, end), best)
    population = [best.copy()] + [mutate(best, window, 1.0, rng)
                                  for _ in range(POPULATION - 1)]
    scale = 1.0
    top = (-math.inf, "", best)
    # until GENERATIONS have run and the best survives and, if the plan
    # started from fails, is as far along the road as the plan was when it
    # failed, at most MAX_GENERATIONS: a window that ends with none would
    # carry a crash into the next, and one that ends behind the plan a car
    # that has all but stopped
    for generation in range(MAX_GENERATIONS):
        results = evaluate_all(base, stage, population, end, pool, work,
                               counter)
        scores = np.array([r[0] for r in results])
        if generation == 0:
            plan = results[0]
            window.brake_times = pacenote_times(stage, plan[3], start, end)
            window.turn_times = pacenote_times(stage, plan[3], start, end,
                                               TURN_IN_BEFORE)
        order = np.argsort(-scores)
        if scores[order[0]] > top[0]:
            top = (float(scores[order[0]]), results[order[0]][1],
                   population[order[0]], results[order[0]][3])
        report(log_file, start, generation, top, scores, counter)
        behind = behind_plan(plan, top)
        if generation >= GENERATIONS - 1 and top[0] > FAILED / 2 and \
                not behind:
            break
        children = [population[i].copy() for i in order[:ELITE]]
        rescue = top[0] <= FAILED / 2
        if not rescue and not behind:
            scale *= SIGMA_DECAY

        def pick():
            entrants = rng.choice(len(population), TOURNAMENT, replace=False)
            return population[max(entrants, key=lambda e: scores[e])]

        while len(children) < POPULATION:
            children.append(mutate(crossover(pick(), pick(), window, rng),
                                   window, scale, rng, rescue))
        population = children
    return top[2], top[0], top[1]


def optimize_window_cma(base, stage, best, start, pool, work, rng, log_file,
                        counter):
    end = start + WINDOW
    window = Window((start, end), best)
    n = len(window.encode(best))
    es = cma.CMAEvolutionStrategy(np.zeros(n), SIGMA0, {
        "popsize": POPULATION, "seed": int(rng.integers(1, 2 ** 31)),
        "integer_variables": window.integers(), "verbose": -9,
        "verb_log": 0, "verb_disp": 0, "tolx": 0, "tolfun": 0,
        "tolflatfitness": MAX_GENERATIONS, "tolstagnation": MAX_GENERATIONS})
    top = (-math.inf, "", best)
    # until GENERATIONS have run and one candidate survives, at most
    # MAX_GENERATIONS: a window that ends with none would carry a crash
    # into the next
    for generation in range(MAX_GENERATIONS):
        # the best schedules that survive so far (at first those started
        # from), so that the mean is drawn to them and not to the fast runs
        # that fail; then the moves that start a slide, about the mean
        injected = [np.zeros(n)] if generation == 0 else \
            [window.encode(top[2])] if top[0] > FAILED / 2 else []
        mean = window.decode(es.mean)
        for k in range(int(round(CMA_MOVES * POPULATION))):
            child = mean.copy()
            move(child, window, rng, (k % 4 + 0.5) / 4, 1.0)
            child.clip()
            injected.append(window.encode(child))
        es.inject(injected, force=True)
        xs = es.ask()
        population = [window.decode(x) for x in xs]
        results = evaluate_all(base, stage, population, end, pool, work,
                               counter)
        scores = np.array([r[0] for r in results])
        # CMA-ES minimizes; it is given the vectors of the knots as decoded,
        # the steps clipped at the limits
        es.tell([window.encode(c) for c in population], list(-scores))
        i = int(np.argmax(scores))
        if scores[i] > top[0]:
            top = (float(scores[i]), results[i][1], population[i])
        report(log_file, start, generation, top, scores, counter,
               "  sigma %.3f" % es.sigma)
        if generation >= GENERATIONS - 1 and top[0] > FAILED / 2:
            break
    return top[2], top[0], top[1]


def smooth_noise(n, step, rng):
    """n samples of noise of unit spread, correlated over MPPI_SMOOTH s by
    a Gaussian kernel at knots step s apart"""
    width = MPPI_SMOOTH / step
    k = np.arange(-int(3 * width), int(3 * width) + 1)
    kernel = np.exp(-0.5 * (k / width) ** 2)
    noise = np.convolve(rng.normal(size=n + 2 * len(k)), kernel, "same")
    noise = noise[len(k):len(k) + n]
    return noise / (np.linalg.norm(kernel) or 1.0)


def optimize_window_mppi(base, stage, best, start, pool, work, rng,
                         log_file, counter):
    """MPPI over the window: the steering and the throttle of the plan
    moved each iteration to the average of samples about it, smooth noise
    and moves that start a slide, weighted by exp(score / MPPI_TEMPERATURE);
    the brake, the handbrake and the gears, which an average would spread
    and weaken, from the sample of the largest weight"""
    end = start + WINDOW
    window = Window((start, end), best)
    ms, mp = window.masks["steer"], window.masks["throttle"]
    plan = best.copy()
    top = (-math.inf, "", best)
    scale = 1.0
    for generation in range(MAX_GENERATIONS):
        population = [plan.copy()]
        while len(population) < POPULATION:
            child = plan.copy()
            child.steer[ms] += MPPI_SIGMA * SCALE["steer"] * scale * \
                smooth_noise(ms.sum(), STEER_STEP, rng)
            child.throttle[mp] += MPPI_SIGMA * SCALE["throttle"] * scale * \
                smooth_noise(mp.sum(), PEDAL_STEP, rng)
            child.brake[mp] *= 1.0 + SCALE["brake"] * scale * \
                smooth_noise(mp.sum(), PEDAL_STEP, rng)
            u = rng.random()
            if u < CMA_MOVES:
                move(child, window, rng, u, CMA_MOVES)
            elif top[0] <= FAILED / 2 and u < CMA_MOVES + RESCUE:
                lift_and_brake(child, window.masks, rng)
            child.clip()
            population.append(child)
        results = evaluate_all(base, stage, population, end, pool, work,
                               counter)
        scores = np.array([r[0] for r in results])
        if generation == 0:
            first = results[0]
            window.turn_times = pacenote_times(stage, first[3], start, end,
                                               TURN_IN_BEFORE)
            window.brake_times = pacenote_times(stage, first[3], start,
                                                end)
        i = int(np.argmax(scores))
        if scores[i] > top[0]:
            top = (float(scores[i]), results[i][1], population[i],
                   results[i][3])
        report(log_file, start, generation, top, scores, counter)
        w = np.exp((scores - scores[i]) / MPPI_TEMPERATURE)
        w /= w.sum()
        # the effective sample size of the weights, 1 / sum(w^2): how many
        # samples the average is in effect over, of those that survived
        line = "    effective samples %.2f of %d that survived" % (
            1.0 / float(np.sum(w ** 2)), int(np.sum(scores > FAILED / 2)))
        print(line, flush=True)
        log_file.write(line + "\n")
        behind = behind_plan(first, top)
        if generation >= GENERATIONS - 1 and top[0] > FAILED / 2 and \
                not behind:
            break
        for name, m in (("steer", ms), ("throttle", mp)):
            getattr(plan, name)[m] = sum(
                wk * getattr(c, name)[m] for wk, c in zip(w, population))
        for name in ("brake", "handbrake", "gear"):
            getattr(plan, name)[mp] = getattr(population[i], name)[mp]
        plan.clip()
        if top[0] > FAILED / 2 and not behind:
            scale *= SIGMA_DECAY
    return top[2], top[0], top[1]


def write_result(base, best, stage, work, path):
    """simulate the best schedules over the whole stage and write the
    model of them next to this script, with paths relative to it"""
    log = run(base, best, TIME_LIMIT, os.path.join(work, "final"))
    t_finish = finish_time(stage, log) if log is not None else None
    span = round(t_finish + RUN_ON, 2) if t_finish else TIME_LIMIT
    m = model(base, best, span, "polo_r_wrc_forest_schedule.csv")

    def relative(value):
        if isinstance(value, dict):
            return {k: relative(v) for k, v in value.items()}
        if isinstance(value, list):
            return [relative(v) for v in value]
        if isinstance(value, str) and value.startswith(ROOT + "/"):
            r = os.path.relpath(value, HERE)
            return r + ("/" if value.endswith("/") else "")
        return value

    m = relative(m)
    m["model_parameters"]["include_directories"] = [".", "../polo_r_wrc"]
    m["model_parameters"]["integrators"][0]["time_step"] = 0.02
    header = """  // The Polo R WRC of ../polo_r_wrc on the gravel stage of
  // ../../assets/environments/forest, its steering, throttle, brakes,
  // handbrake and gears set by schedules of time found to be the fastest
  // through the stage without hitting a tree (written by
  // optimize_polo_r_wrc_forest.py); the driver controller only observes,
  // reporting the station of the car along the road."""
    text = json.dumps(m, indent=2).replace("{\n", "{\n" + header + "\n", 1)
    with open(path, "w") as f:
        f.write(text + "\n")
    return t_finish, path


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--work", required=True,
                        help="directory for the runs and the state")
    parser.add_argument("--workers", type=int, default=30)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--method", choices=("ga", "cma", "mppi"),
                        default="ga")
    parser.add_argument("--seed-log", help="the log of a run of the stage "
                        "whose inputs the search starts from (by default "
                        "that of polo_r_wrc_forest.json)")
    parser.add_argument("--output", default=os.path.join(
        HERE, "polo_r_wrc_forest_schedule.json"))
    args = parser.parse_args()
    work = os.path.abspath(args.work)
    os.makedirs(work, exist_ok=True)
    base = base_model()
    stage = Stage(base, work)
    state_path = os.path.join(work, "state.json")
    rng = np.random.default_rng(args.seed)
    if os.path.exists(state_path):
        with open(state_path) as f:
            state = json.load(f)
        best = Schedules.from_json(state["best"])
        start, counter = state["start"], [state["runs"]]
    else:
        best, start, counter = seed_schedules(args.seed_log), 0.0, [0]
    # the seed schedules and the log of their run, to retime the knots past
    # the windows searched so far by
    seed = seed_schedules(args.seed_log)
    reference = read_log(args.seed_log or
                         os.path.join(HERE, "polo_r_wrc_forest.csv"))
    log_file = open(os.path.join(work, "progress.log"), "a")
    print("road %.1f m, %d obstacles within 25 m of it" %
          (stage.length, len(stage.xy)), flush=True)
    best = realign_report(base, best, seed, reference, start, work, log_file)
    with concurrent.futures.ThreadPoolExecutor(args.workers) as pool:
        while start < TIME_LIMIT - WINDOW:
            optimize_window = {"ga": optimize_window_ga,
                               "cma": optimize_window_cma,
                               "mppi": optimize_window_mppi}[args.method]
            best, value, why = optimize_window(base, stage, best, start, pool,
                                               work, rng, log_file, counter)
            start += STRIDE
            # the knots past the windows searched so far, retimed to where
            # the car is along the road (see realign)
            best = realign_report(base, best, seed, reference, start, work,
                                  log_file)
            with open(state_path, "w") as f:
                json.dump({"best": best.to_json(), "start": start,
                           "runs": counter[0], "score": value,
                           "why": why}, f)
            if why.startswith("finished"):
                t = float(why.split()[2])
                # the last windows: until one holds the end of the run,
                # RUN_ON s after the finish
                if start - STRIDE + WINDOW >= t + RUN_ON:
                    break
    t_finish, path = write_result(base, best, stage, work,
                                  os.path.abspath(args.output))
    line = "finish %s s; wrote %s; %d runs" % (
        "%.3f" % t_finish if t_finish else "not reached", path, counter[0])
    print(line, flush=True)
    log_file.write(line + "\n")


if __name__ == "__main__":
    main()
