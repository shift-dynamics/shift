#!/usr/bin/env python3
"""The fastest run of the Polo R WRC through the forest stage with the driver
controller of make_polo_r_wrc_forest.py, found by an evolution strategy
(CMA-ES, from the cma package) over the line it follows and its settings.

The driver steers by pure pursuit along a path and sets the throttle, brakes
and gear for the speed profile of the path, from its lateral_acceleration,
acceleration and deceleration (see shift/core/controller/driver.h). This is
the first step of the two-step method of Kapania, Subosits and Gerdes (A
Sequential Two-Step Algorithm for Fast Generation of Vehicle Racing
Trajectories, 2016): the speed profile the grip allows along a line. In
place of their second step, the line through the bends that shortens the
time, the line here is the middle of the road moved across it by an offset
that is a cubic spline through knots every LINE_STEP m (at most LINE_LIMIT m
from the middle, 0 at the start), and the offsets and the driver's settings
are searched together by running the car through the whole stage in
shift-cli. The line runs on straight past the end of the road, so that the
driver does not slow for the end of it before the finish line.

With --line least-curvature the line is instead fixed: the line of least
curvature within the gravel (Line.least_curvature), the one that
straightens the bends the most, on the middle of the road at the start and
at the finish line, and only the driver's settings are searched.

The racing driver ("racer") drives; the driver controller "driver" stays in
the model on the middle of the road with its outputs unconnected and reports
the station of the car along the road, by which a run is scored as by
optimize_polo_r_wrc_forest.py: the time it crosses the finish line, or, if
it does not, how far it got; a run that hits a tree, a sapling or a rock,
rolls over or turns back ranks below every run that does not.

Writes polo_r_wrc_forest_line.json next to this script: the stage with the
best line and settings. The runs go in a work directory (--work), with the
state after each generation, from which the search resumes.

    python3 optimize_polo_r_wrc_forest_line.py --work <dir> [--workers 7]
        [--line spline|least-curvature] [--start-settings <best.json>]
        [--output <file>]
"""

import argparse
import concurrent.futures
import copy
import json
import math
import os
import pickle
import shutil
import subprocess
import sys

import cma
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import lsq_linear

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_polo_r_wrc_forest  # noqa: E402
import optimize_polo_r_wrc_forest as schedule_search  # noqa: E402

# the line: offsets (m, to the right of the middle) every LINE_STEP m along
# the road, at most LINE_LIMIT m (the gravel is 3.4 m each side of the
# middle and the car 0.92 m each side of its own), and the straight run on
# past the end of the road (m)
LINE_STEP = 20.0
LINE_LIMIT = 3.0
RUN_ON = 200.0
# the line of least curvature (--line least-curvature): at most
# CURVATURE_LIMIT m from the middle (the gravel's edge less the car's half
# width and a margin), on the middle at the start and at the finish line
CURVATURE_LIMIT = 2.2
# the settings of the driver searched, each its value to start from, the
# spread of a step of 1 and its bounds
SETTINGS = {
    "lateral_acceleration": (4.5, 1.0, 3.0, 10.0),
    "acceleration": (4.0, 1.0, 2.0, 10.0),
    "deceleration": (5.0, 1.0, 3.0, 12.0),
    "lookahead_distance": (4.0, 1.0, 1.5, 10.0),
    "lookahead_time": (0.35, 0.1, 0.05, 1.0),
    "speed": (30.0, 3.0, 15.0, 45.0),
    "throttle_gain": (0.4, 0.2, 0.1, 3.0),
    "brake_gain": (0.25, 0.1, 0.05, 2.0),
}
# CMA-ES
POPULATION = 14
SIGMA0 = 0.5
GENERATIONS = 200
# the time simulated (s): past the finish of the driver controller (57 s)
TIME_SPAN = 66.0


class Line:
    """the middle of the road and the knots of the offsets along it"""

    def __init__(self, driver):
        xy = np.array(driver["path"]).reshape(-1, 2)
        d = np.diff(xy, axis=0)
        self.xy = xy
        self.s = np.concatenate([[0.0],
                                 np.cumsum(np.linalg.norm(d, axis=1))])
        tangent = np.gradient(xy, self.s, axis=0)
        tangent /= np.linalg.norm(tangent, axis=1)[:, None]
        # to the right of the direction of travel, with x forward, y right
        # and z down
        self.right = np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)
        self.end = tangent[-1]
        self.knots = np.arange(0.0, self.s[-1], LINE_STEP)
        self.knots = np.append(self.knots, self.s[-1])

    def path(self, offsets, fixed=None):
        """the path of the racer, [x0, y0, x1, y1, ...], for the offsets of
        the knots after the first, added to the offsets fixed at each point
        of the middle of the road (if given)"""
        spline = CubicSpline(self.knots, np.concatenate([[0.0], offsets]))
        offset = spline(self.s)
        if fixed is not None:
            offset = offset + fixed
        moved = self.xy + offset[:, None] * self.right
        run_on = moved[-1] + np.arange(2.0, RUN_ON + 1e-9, 2.0)[:, None] * \
            self.end
        return [round(float(v), 3)
                for v in np.concatenate([moved, run_on]).ravel()]

    def least_curvature(self, limit, finish):
        """the offsets at the points of the middle of the road of the line
        of least curvature within limit m of the middle, on it at the start
        and at the station finish: the sum of the squares of the second
        differences of the points of the line, which are evenly spaced
        along the middle (to the curvature times the spacing squared, for
        offsets small beside the radius of the bends), is least (Braghin,
        Cheli, Melzi and Sabbioni, Race driver model, Computers and
        Structures 86, 2008); it is linear in the offsets, so that this is
        a least squares problem with bounds"""
        n = len(self.s)
        a = np.zeros((2 * (n - 2), n))
        b = np.zeros(2 * (n - 2))
        for i in range(1, n - 1):
            for j, w in ((i - 1, 1.0), (i, -2.0), (i + 1, 1.0)):
                a[2 * (i - 1):2 * i, j] += w * self.right[j]
                b[2 * (i - 1):2 * i] -= w * self.xy[j]
        low, high = np.full(n, -limit), np.full(n, limit)
        held = np.zeros(n, bool)
        held[:2] = True
        held[np.argmin(np.abs(self.s - finish))] = True
        low[held], high[held] = -1e-9, 1e-9
        return lsq_linear(a, b, bounds=(low, high)).x


class Candidate:
    """a vector of CMA-ES as the offsets and the settings, or, for a line
    fixed by offsets at each point of the middle of the road, the settings
    only; the settings about those given (start), or about the starting
    values of SETTINGS"""

    def __init__(self, line, fixed=None, start=None):
        self.line = line
        self.fixed = fixed
        self.start = start or {}
        self.n_offsets = 0 if fixed is not None else len(line.knots) - 1

    def size(self):
        return self.n_offsets + len(SETTINGS)

    def decode(self, x):
        if self.fixed is not None:
            offsets = np.zeros(len(self.line.knots) - 1)
        else:
            offsets = np.clip(x[:self.n_offsets], -LINE_LIMIT, LINE_LIMIT)
        settings = {}
        for i, (name, (start, spread, low, high)) in enumerate(
                SETTINGS.items()):
            start = self.start.get(name, start)
            settings[name] = float(np.clip(
                start + spread * x[self.n_offsets + i], low, high))
        return offsets, settings


def base_model():
    """the stage of make_polo_r_wrc_forest.py with the driver renamed racer,
    and an observing driver on the middle of the road"""
    model, header = make_polo_r_wrc_forest.build_model()
    model = schedule_search.absolute(model)
    params = model["model_parameters"]
    params["include_directories"] = [
        HERE, os.path.join(schedule_search.ROOT, "examples", "polo_r_wrc")]
    params["integrators"][0]["time_step"] = schedule_search.LOG_STEP
    params["integrators"][0]["time_span"] = TIME_SPAN
    controllers = model["assemblies"][0]["controllers"]
    racer = next(c for c in controllers if c["type"] == "driver")
    observer = copy.deepcopy(racer)
    racer["name"] = "racer"
    for c in controllers:
        if c.get("component") == "driver":
            c["component"] = "racer"
    # the observer first, so that the stage finds the road from it
    model["assemblies"][0]["controllers"] = [observer] + controllers
    return model, header


def model(base, candidate, x, logfile):
    offsets, settings = candidate.decode(x)
    m = copy.deepcopy(base)
    m["model_parameters"]["loggers"][0]["csv_logfile"] = logfile
    racer = next(c for c in m["assemblies"][0]["controllers"]
                 if c["name"] == "racer")
    racer["path"] = candidate.line.path(offsets, candidate.fixed)
    racer.update(settings)
    return m


def evaluate(base, stage, candidate, x, directory):
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, "model.json")
    with open(path, "w") as f:
        json.dump(model(base, candidate, x, "run.csv"), f)
    try:
        subprocess.run([schedule_search.SHIFT_CLI, "-v", "warning", path],
                       cwd=directory, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, timeout=60.0 + 3.0 *
                       TIME_SPAN)
        log = schedule_search.read_log(os.path.join(directory, "run.csv"))
    except Exception:
        log = None
    shutil.rmtree(directory, ignore_errors=True)
    if log is None or len(log["time"]) < 2:
        return schedule_search.FAILED, "no log"
    return schedule_search.score(stage, log, TIME_SPAN)


def write_result(base, header, candidate, x, path):
    """the model of the best line and settings, with paths relative to
    this directory"""
    m = model(base, candidate, x, "polo_r_wrc_forest_line.csv")

    def relative(value):
        if isinstance(value, dict):
            return {k: relative(v) for k, v in value.items()}
        if isinstance(value, list):
            return [relative(v) for v in value]
        if isinstance(value, str) and \
                value.startswith(schedule_search.ROOT + "/"):
            r = os.path.relpath(value, HERE)
            return r + ("/" if value.endswith("/") else "")
        return value

    m = relative(m)
    m["model_parameters"]["include_directories"] = [".", "../polo_r_wrc"]
    m["model_parameters"]["integrators"][0]["time_step"] = 0.02
    header = """  // The Polo R WRC of ../polo_r_wrc on the gravel stage of
  // ../../assets/environments/forest, driven by the driver controller
  // "racer" along a line across the road and with settings found to be the
  // fastest through the stage without hitting a tree (written by
  // optimize_polo_r_wrc_forest_line.py); the driver controller "driver"
  // only observes, reporting the station of the car along the road."""
    text = json.dumps(m, indent=2).replace("{\n", "{\n" + header + "\n", 1)
    with open(path, "w") as f:
        f.write(text + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--work", required=True,
                        help="directory for the runs and the state")
    parser.add_argument("--workers", type=int, default=7)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--output", default=os.path.join(
        HERE, "polo_r_wrc_forest_line.json"))
    parser.add_argument("--line", choices=("spline", "least-curvature"),
                        default="spline", help="search the offsets of a "
                        "spline across the road with the settings, or only "
                        "the settings along the line of least curvature")
    parser.add_argument("--start-settings", help="a best.json of an earlier "
                        "search whose settings the search starts from")
    args = parser.parse_args()
    work = os.path.abspath(args.work)
    os.makedirs(work, exist_ok=True)
    base, header = base_model()
    stage = schedule_search.Stage(base, work)
    driver = next(c for c in base["assemblies"][0]["controllers"]
                  if c["name"] == "driver")
    line = Line(driver)
    start = None
    if args.start_settings:
        with open(args.start_settings) as f:
            start = json.load(f)["settings"]
    fixed = line.least_curvature(CURVATURE_LIMIT, stage.finish) \
        if args.line == "least-curvature" else None
    candidate = Candidate(line, fixed, start)
    n = candidate.size()
    state_path = os.path.join(work, "state.pickle")
    if os.path.exists(state_path):
        with open(state_path, "rb") as f:
            es, top, generation, runs = pickle.load(f)
    else:
        es = cma.CMAEvolutionStrategy(np.zeros(n), SIGMA0, {
            "popsize": POPULATION, "seed": args.seed, "verbose": -9,
            "verb_log": 0, "verb_disp": 0, "tolx": 0, "tolfun": 0,
            "tolflatfitness": GENERATIONS, "tolstagnation": GENERATIONS})
        top, generation, runs = (-math.inf, "", np.zeros(n)), 0, 0
    log_file = open(os.path.join(work, "progress.log"), "a")
    print("road %.1f m, finish at %.1f m; %d offsets and %d settings" %
          (stage.length, stage.finish, candidate.n_offsets, len(SETTINGS)),
          flush=True)
    with concurrent.futures.ThreadPoolExecutor(args.workers) as pool:
        while generation < GENERATIONS:
            # the line and settings started from are run first
            if generation == 0:
                es.inject([np.zeros(n)], force=True)
            elif top[0] > schedule_search.FAILED / 2:
                es.inject([top[2]], force=True)
            xs = es.ask()
            jobs = []
            for x in xs:
                runs += 1
                jobs.append(pool.submit(
                    evaluate, base, stage, candidate, x,
                    os.path.join(work, "runs", "%06d" % runs)))
            results = [j.result() for j in jobs]
            scores = np.array([r[0] for r in results])
            es.tell(xs, list(-scores))
            i = int(np.argmax(scores))
            if scores[i] > top[0]:
                top = (float(scores[i]), results[i][1], np.array(xs[i]))
            failed = int(np.sum(scores <= schedule_search.FAILED / 2))
            line = ("generation %3d  best %8.2f (%s)  this %s  failed %2d/%d"
                    "  runs %d  sigma %.3f" %
                    (generation, top[0], top[1], results[i][1], failed,
                     len(xs), runs, es.sigma))
            print(line, flush=True)
            log_file.write(line + "\n")
            log_file.flush()
            generation += 1
            with open(state_path, "wb") as f:
                pickle.dump((es, top, generation, runs), f)
            if top[0] > schedule_search.FAILED / 2:
                write_result(base, header, candidate, top[2],
                             os.path.abspath(args.output))
                offsets, settings = candidate.decode(top[2])
                with open(os.path.join(work, "best.json"), "w") as f:
                    json.dump({"why": top[1], "settings": settings,
                               "offsets": offsets.tolist()}, f, indent=1)


if __name__ == "__main__":
    main()
