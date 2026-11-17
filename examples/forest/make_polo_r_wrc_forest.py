#!/usr/bin/env python3
"""The Polo R WRC of ../polo_r_wrc driven along the gravel stage of
../../assets/environments/forest.

Writes polo_r_wrc_forest.json, next to this script: the Polo on the stage,
its steering, throttle, brakes and gears set by a driver controller that
follows the middle of the road.
"""

import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "assets", "environments",
                                "forest"))
import make_forest  # noqa: E402


def read_json(path):
    """a model file, without its // comments"""
    with open(path) as f:
        return json.loads(re.sub(r"^\s*//.*$", "", f.read(), flags=re.M))


# the Polo's steering: the rack moves 0.135 m per radian of the front
# wheels (the steering arm, from the axle to the outer ball joint of the
# tie rod), and a positive rack position (to the right) turns it left
RACK_PER_RADIAN = 0.135
# the gears: up at about 720 rad/s of the engine, past its peak power
SHIFT_SPEEDS = [17.0, 21.0, 25.5, 31.0, 37.5]
TIME_SPAN = 66.0


def build_model():
    """the model of polo_r_wrc_forest.json, and its header comment"""
    model = read_json(os.path.join(HERE, "..", "polo_r_wrc",
                                   "polo_r_wrc.json"))
    parts = make_forest.forest_parts(HERE)
    params = model["model_parameters"]
    params["include_directories"] = [".", "../polo_r_wrc"]
    params["integrators"][0]["time_span"] = TIME_SPAN
    params["loggers"][0]["csv_logfile"] = "polo_r_wrc_forest.csv"
    # the stage in place of the Polo's: the environment's sun and sky light
    # it, not the lamps over the start
    model.pop("lights", None)
    model["frames"] = parts["frames"]
    for key in ("environment", "surface_materials", "terrain", "scenery"):
        model[key] = parts[key]
    assembly = model["assemblies"][0]
    chassis = next(b for b in assembly["bodies"] if b["name"] == "chassis")
    # the frame the driver steers by, at the middle of the rear axle
    chassis.setdefault("frames", []).append(
        {"name": "rear_axle", "T": [-1.205, 0.0, -0.314]})
    # the driver in place of the schedules of the route; the clutch is let
    # in at the start as before, the stop lamps follow the brake, and the
    # handbrake stays off (the driver does not pull it)
    controllers = [c for c in assembly["controllers"]
                   if c["name"] in ("clutch_pedal", "steering",
                                    "brake_lights", "handbrake",
                                    "rear_drive")]
    clutch = next(c for c in controllers if c["name"] == "clutch_pedal")
    clutch["points"] = [0, 0, 0.3, 0, 1.0, 1.0]
    steering = next(c for c in controllers if c["name"] == "steering")
    lamps = next(c for c in controllers if c["name"] == "brake_lights")
    lamps["component"] = "driver"
    lamps["output"] = "brake"
    brakes = ["force_modules/drive/brakes/" + side + "_" + end + "/pressure"
              for side in ("left", "right") for end in ("front", "rear")]
    angle = 0.5
    # the bends taken at 4.5 m/s^2, short of the grip of the gravel, so that
    # the car slides only where the stage makes it, in the off-camber left
    # over a crest; at 5 m/s^2 it slides in the bends of the descent after it
    # too, and spins
    assembly["controllers"] = [
        make_forest.driver(["chassis", "rear_axle"], 2.41, speed=30.0,
               lateral_acceleration=4.5, acceleration=4.0, deceleration=5.0,
               lookahead_distance=4.0, lookahead_time=0.35,
               max_steering_angle=angle, throttle_gain=0.4, brake_gain=0.25,
               shift_speeds=SHIFT_SPEEDS),
        {"type": "map", "name": "throttle", "component": "driver",
         "output": "throttle",
         "targets": ["force_modules/drive/engine/throttle"]},
        {"type": "map", "name": "brake_pedal", "component": "driver",
         "output": "brake", "targets": brakes},
        {"type": "map", "name": "gears", "component": "driver",
         "output": "gear", "targets": ["force_modules/drive/transmission/gear"]},
        {"type": "map", "name": "steering_wheel", "component": "driver",
         "output": "steering",
         "points": [-angle, angle * RACK_PER_RADIAN,
                    angle, -angle * RACK_PER_RADIAN],
         "targets": ["controllers/steering/target"]},
        clutch, steering, lamps] + [
        c for c in controllers if c["name"] in ("handbrake", "rear_drive")]
    header = """  // The Polo R WRC of ../polo_r_wrc on the gravel stage through a conifer
  // forest of ../../assets/environments/forest (written by
  // make_polo_r_wrc_forest.py): a driver controller steers it along the
  // middle of the road and sets the throttle, brakes and gears for the
  // speed the bends allow; trees, shrubs, grass and rocks are the stage's
  // scenery, from ../../assets/scenery."""
    return model, header


def write_polo():
    model, header = build_model()
    text = json.dumps(model, indent=2)
    text = text.replace("{\n", "{\n" + header + "\n", 1)
    with open(os.path.join(HERE, "polo_r_wrc_forest.json"), "w") as f:
        f.write(text + "\n")


if __name__ == "__main__":
    write_polo()
    print("wrote polo_r_wrc_forest.json in", HERE)
