#!/usr/bin/env python3
"""The Strandbeest of ../strandbeest walking on the beach of
../../assets/environments/beach.

Writes strandbeest_beach.json, next to this script: strandbeest.json on the
beach, its feet in the deformable sand of the beach's strip, the sea beside
it.
"""

import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "assets", "environments",
                                "beach"))
import make_beach  # noqa: E402


def read_json(path):
    """a model file, without its // comments"""
    with open(path) as f:
        return json.loads(re.sub(r"^\s*//.*$", "", f.read(), flags=re.M))


def write_strandbeest():
    model = read_json(os.path.join(HERE, "..", "strandbeest",
                                   "strandbeest.json"))
    parts = make_beach.beach_parts(HERE)
    # the beach in place of the walker's sand square, plane and lights:
    # the environment's sun and sky light it
    for key in ("lights", "collision_geometry"):
        model.pop(key, None)
    model["frames"] = parts["frames"]
    for key in ("environment", "surface_materials", "terrain", "scenery"):
        model[key] = parts[key]
    model["model_parameters"]["include_directories"] = [".", "../strandbeest"]
    assembly = model["assemblies"][0]
    assembly["bodies"] = [b for b in assembly["bodies"]
                          if b["name"] != "beach"]
    assembly["joints"] = [j for j in assembly["joints"]
                          if j["name"] != "beach_fixed"]
    feet = {g["name"]: b["name"] for b in assembly["bodies"]
            for g in b.get("collision_geometry", [])
            if g["name"].endswith("_foot")}
    # the feet in the sand: the soil carries them
    assembly["force_modules"] = [{"type": "soil_contact",
                                  "name": "soil_contact",
                                  "bodies": sorted(set(feet.values())),
                                  "geometry": sorted(feet)}]
    # standing as on the plane, on the sand at the middle of the strip
    ground = float(make_beach.height(np.array(0.0), np.array(0.0)))
    for joint in assembly["joints"]:
        if joint["name"] == "chassis_float":
            joint["initial_displacement"]["T"][2] -= ground
    header = """  // The Strandbeest of ../strandbeest on the sand beach of
  // ../../assets/environments/beach (written by make_strandbeest_beach.py):
  // its feet in the deformable wet sand of the strip "strip", the scenery
  // around it the basic terrain "beach", the sea the environment's water."""
    text = json.dumps(model, indent=2)
    text = text.replace("{\n", "{\n" + header + "\n", 1)
    with open(os.path.join(HERE, "strandbeest_beach.json"), "w") as f:
        f.write(text + "\n")


if __name__ == "__main__":
    write_strandbeest()
    print("wrote strandbeest_beach.json in", HERE)
