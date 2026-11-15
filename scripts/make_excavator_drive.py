#!/usr/bin/env python3
"""Write the excavator that drives, examples/excavator/excavator_drive.json,
with its tracks, sprockets, idlers and rollers as STEP parts taken from the
undercarriage mesh of the excavator, examples/excavator/undercarriage.obj.

The mesh draws the whole undercarriage as one body. Every part of the
tracks in it is a prism along the axles (the mesh's x): a cross section in
the plane of the track (y forward... z up) swept between two planes of
constant x. The script slices the mesh at the middle of each of those slabs
and so recovers the cross sections exactly, as loops of the mesh's vertices,
and writes each part as solids extruded from them, with the runs of
vertices of the facets of a circle written as an arc through them
(make_track_step.extrude):

* excavator_track_link.step: a link of the track, the solids of its seven
  slabs across the track: the shoe (one grouser, its edges curled over the
  next shoe's), the outer and the inner link plate of each side welded to
  it, and the bushing. Its origin at the bushing's center, x towards the next
  link's bushing (the pitch, 210 mm, away), y along the bushing, z out of
  the track loop.
* excavator_sprocket.step: the drive sprocket, 23 teeth, the bushings in
  every second seat.
* excavator_idler.step: the front idler, a hub between two flanges.
* excavator_track_roller.step and excavator_carrier_roller.step: the
  rollers, under the bottom run of the track (with flanges either side of
  the link plates) and under the top run.

The wheels' origin is their center, z along the axle, x forward and y up
in the mesh (the mesh's y and z). The undercarriage frame that is left,
the car body and the track frames, is written as
examples/excavator/undercarriage_frame.obj: the mesh's faces without those
on the surface of a part.

The model places every part where the mesh has it: the 46 links of each
track at the bushings' centers in the mesh, which sit in the sprocket's
seats, on a circle about the idler, on the treads of the track rollers and
over the carrier rollers. Its upper structure, with the hydraulic cylinders
of the boom, stick and bucket, is that of examples/excavator/excavator.json.

Usage: make_excavator_drive.py [examples/excavator directory]
"""

import collections
import json
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_track_step as track_step  # noqa: E402

SOURCE = 'scripts/make_excavator_drive.py'
STEEL_DENSITY = 7850.0      # kg/m^3, of every part
PITCH = 0.21                # m, of the mesh's track
TOLERANCE = 1e-4            # m, of a face on a part's surface
ARC_TOLERANCE = 2e-5        # m, of a vertex off the arc through its run
PLANE_MERGE = 5e-4          # m: planes of x closer are one
COLLINEAR = 5e-6            # m, of a vertex off its neighbors' line
INSET = 2e-3                # m, inside a face, to tell whose it is
SECTION_STEP = 2e-4         # m, between the cross sections that tell it


# ---------------------------------------------------------------------------
# the mesh

class Mesh:
    """the triangles of an OBJ file, with the face each one came from"""

    def __init__(self, path):
        self.lines = open(path).read().split('\n')
        vertices, triangles, faces = [], [], []
        for i, line in enumerate(self.lines):
            if line.startswith('v '):
                vertices.append([float(x) for x in line.split()[1:4]])
            elif line.startswith('f '):
                idx = [int(t.split('/')[0]) - 1 for t in line.split()[1:]]
                for k in range(1, len(idx) - 1):
                    triangles.append((idx[0], idx[k], idx[k + 1]))
                    faces.append(i)
        self.vertices = np.array(vertices)
        self.triangles = self.vertices[np.array(triangles)]
        self.faces = np.array(faces)  # the line of each triangle's face

    def region(self, lo, hi):
        """the triangles all inside the box lo..hi"""
        t = self.triangles
        inside = np.all((t.min(1) >= np.array(lo) - 1e-9) &
                        (t.max(1) <= np.array(hi) + 1e-9), axis=1)
        return t[inside]


def planes(triangles):
    """the values of x at the ends of the slabs of the prisms: those of the
    triangles' vertices, closer than PLANE_MERGE merged"""
    xs = sorted(set(np.round(triangles[:, :, 0].ravel(), 5)))
    merged = []
    for x in xs:
        if merged and x - merged[-1][-1] < PLANE_MERGE:
            merged[-1].append(x)
        else:
            merged.append([x])
    return merged


def section(triangles, x, key=1e-6):
    """the closed loops of the cross section at x, in (y, z), counterclockwise
    for faces wound outward. Touching solids are kept apart by turning
    sharpest left where loops meet."""
    segments = []
    crossing = ((triangles[:, :, 0].min(1) < x) &
                (triangles[:, :, 0].max(1) > x))
    for t in triangles[crossing]:
        n = np.cross(t[1] - t[0], t[2] - t[0])
        if np.linalg.norm(n[1:]) < 1e-12:
            continue
        d = t[:, 0] - x
        points = []
        for i in range(3):
            a, b = t[i], t[(i + 1) % 3]
            if (d[i] < 0) != (d[(i + 1) % 3] < 0):
                s = d[i] / (d[i] - d[(i + 1) % 3])
                points.append(a[1:] + s * (b[1:] - a[1:]))
        if len(points) != 2 or np.linalg.norm(points[0] - points[1]) < 1e-9:
            continue
        a, b = points
        if np.dot(b - a, np.array([-n[2], n[1]])) < 0:
            a, b = b, a
        segments.append((a, b))

    def k(p):
        return (round(p[0] / key), round(p[1] / key))

    starting = collections.defaultdict(list)
    for i, (a, b) in enumerate(segments):
        starting[k(a)].append(i)
    used = [False] * len(segments)
    loops = []
    for i in range(len(segments)):
        if used[i]:
            continue
        used[i] = True
        loop, current = [segments[i][0]], i
        while True:
            a, b = segments[current]
            if k(b) == k(loop[0]):
                break
            candidates = [j for j in starting[k(b)]
                          if not used[j] and k(segments[j][1]) != k(a)]
            if not candidates:
                loop = None
                break
            incoming = (b - a) / np.linalg.norm(b - a)

            def turn(j):
                d = segments[j][1] - segments[j][0]
                d = d / np.linalg.norm(d)
                return math.atan2(incoming[0] * d[1] - incoming[1] * d[0],
                                  np.dot(incoming, d))
            current = max(candidates, key=turn)
            used[current] = True
            loop.append(b)
        if loop is not None and len(loop) > 2:
            loops.append(clean(np.array(loop)))
    return loops


def clean(loop):
    """without repeated points and the points within straight runs (the
    mesh's facets carry vertices of their neighbors along their edges)"""
    out = []
    for p in loop:
        if not out or np.linalg.norm(p - out[-1]) > 1e-7:
            out.append(p)
    if np.linalg.norm(out[0] - out[-1]) < 1e-7:
        out.pop()
    changed = True
    while changed and len(out) > 3:
        changed = False
        for i in range(len(out)):
            a, b, c = out[i - 1], out[i], out[(i + 1) % len(out)]
            u, v = b - a, c - b
            w = c - a
            # b within the rounding of the mesh's coordinates of the line
            # from a to c
            if (abs(u[0] * w[1] - u[1] * w[0]) < COLLINEAR * np.linalg.norm(w)
                    and np.dot(u, v) > 0):
                del out[i]
                changed = True
                break
    return np.array(out)


def area(loop):
    a, b = loop[:, 0], loop[:, 1]
    return 0.5 * np.sum(a * np.roll(b, -1) - np.roll(a, -1) * b)


def contains(loop, point):
    """point inside the loop (even-odd)"""
    inside = False
    for i in range(len(loop)):
        a, b = loop[i - 1], loop[i]
        if (a[1] > point[1]) != (b[1] > point[1]):
            x = a[0] + (point[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            if x > point[0]:
                inside = not inside
    return inside


def fit_circle(points):
    """center and radius of the circle through points, least squares"""
    a = np.c_[2.0 * points, np.ones(len(points))]
    s = np.linalg.lstsq(a, (points ** 2).sum(1), rcond=None)[0]
    return s[:2], math.sqrt(s[2] + s[:2] @ s[:2])


# ---------------------------------------------------------------------------
# a cross section as lines and arcs

def circle_through(a, b, c):
    """center of the circle through three points, None if they are on a
    line"""
    d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) +
               c[0] * (a[1] - b[1]))
    if abs(d) < 1e-14:
        return None
    sa, sb, sc = a @ a, b @ b, c @ c
    return np.array([(sa * (b[1] - c[1]) + sb * (c[1] - a[1]) +
                      sc * (a[1] - b[1])) / d,
                     (sa * (c[0] - b[0]) + sb * (a[0] - c[0]) +
                      sc * (b[0] - a[0])) / d])


def is_arc(points):
    """the points (at least four) are the facets of an arc: on a circle,
    turning the same way by less than 30 degrees at each"""
    if len(points) < 4:
        return False
    center = circle_through(points[0], points[len(points) // 2], points[-1])
    if center is None:
        return False
    r = np.linalg.norm(points[0] - center)
    if r > 1.0 or np.abs(np.linalg.norm(points - center, axis=1) -
                         r).max() > ARC_TOLERANCE:
        return False
    signs = set()
    for i in range(1, len(points) - 1):
        u, v = points[i] - points[i - 1], points[i + 1] - points[i]
        cross = u[0] * v[1] - u[1] * v[0]
        if abs(math.atan2(cross, u @ v)) > math.radians(30.0):
            return False
        signs.add(cross > 0)
    return len(signs) == 1


def mm(p):
    """a point in m as a tuple of floats in mm, for the STEP writer"""
    return tuple(float(1000.0 * c) for c in p)


def segments(loop):
    """the loop as make_track_step segments, in mm: ('line', start) and
    ('arc', start, center, ccw), arcs through each run of four or more
    vertices on a circle. A loop all on a circle is a full circle."""
    n = len(loop)
    center, radius = fit_circle(loop)
    if n >= 8 and np.abs(np.linalg.norm(loop - center, axis=1) -
                         radius).max() < ARC_TOLERANCE:
        return track_step.circle_loop(mm(center), float(1000.0 * radius),
                                      bool(area(loop) > 0))
    # start at a vertex no arc runs through
    start = 0
    for i in range(n):
        run = np.array([loop[(i + k) % n] for k in (-2, -1, 0, 1, 2)])
        if not is_arc(run[:4]) and not is_arc(run[1:]) and \
                not is_arc(run[2:] if len(run[2:]) >= 4 else run):
            start = i
            break
    points = [loop[(start + k) % n] for k in range(n)] + [loop[start]]
    out = []
    i = 0
    while i < n:
        j = i + 3
        if j <= n and is_arc(np.array(points[i:j + 1])):
            while j + 1 <= n and is_arc(np.array(points[i:j + 2])):
                j += 1
            a, b = points[i], points[j]
            center = circle_through(a, points[(i + j) // 2], b)
            # on the bisector of a and b, so both ends are on the arc
            mid = 0.5 * (a + b)
            along = np.array([-(b - a)[1], (b - a)[0]])
            along /= np.linalg.norm(along)
            center = mid + along * ((center - mid) @ along)
            u, v = points[i + 1] - a, points[i + 2] - points[i + 1]
            out.append(('arc', mm(a), mm(center),
                        bool(u[0] * v[1] - u[1] * v[0] > 0)))
            i = j
        else:
            out.append(('line', mm(points[i])))
            i += 1
    return out


# ---------------------------------------------------------------------------
# mass properties of prisms

def prism_mass(loop, t0, t1):
    """mass, first moments and second moments (in the part's coordinates,
    the loop in (a, b), extruded along t from t0 to t1) of steel:
    m, [Sa, Sb, St], [[Saa, Sab, Sat], [Sab, Sbb, Sbt], [Sat, Sbt, Stt]]
    (integrals of the coordinates and their products over the mass)"""
    a, b = loop[:, 0], loop[:, 1]
    a1, b1 = np.roll(a, -1), np.roll(b, -1)
    c = a * b1 - a1 * b
    A = 0.5 * c.sum()
    Ia = ((a + a1) * c).sum() / 6.0
    Ib = ((b + b1) * c).sum() / 6.0
    Iaa = ((a * a + a * a1 + a1 * a1) * c).sum() / 12.0
    Ibb = ((b * b + b * b1 + b1 * b1) * c).sum() / 12.0
    Iab = ((a * b1 + 2 * a * b + 2 * a1 * b1 + a1 * b) * c).sum() / 24.0
    L = t1 - t0
    T1 = 0.5 * (t1 * t1 - t0 * t0)
    T2 = (t1 ** 3 - t0 ** 3) / 3.0
    r = STEEL_DENSITY
    m = r * A * L
    first = np.array([r * Ia * L, r * Ib * L, r * A * T1])
    second = r * np.array([[Iaa * L, Iab * L, Ia * T1],
                           [Iab * L, Ibb * L, Ib * T1],
                           [Ia * T1, Ib * T1, A * T2]])
    return m, first, second


class Part:
    """the solids of a part: (name, loops in (a, b), t0, t1, rgb), the
    profile in the plane of the part's axes a and b, extruded along t"""

    def __init__(self, name, axes):
        self.name = name
        self.axes = axes  # the part's (x, y, z) indices of a, b and t
        self.solids = []

    def add(self, name, loop, t0, t1, rgb):
        self.solids.append((name, loop, t0, t1, rgb))

    def mass(self):
        """mass, center of mass and inertia about it, in the part's
        coordinates"""
        m, first, second = 0.0, np.zeros(3), np.zeros((3, 3))
        for _, loop, t0, t1, _ in self.solids:
            mi, fi, si = prism_mass(loop, t0, t1)
            m += mi
            first += fi
            second += si
        # from (a, b, t) to the part's (x, y, z)
        order = np.argsort(self.axes)
        first = first[order]
        second = second[np.ix_(order, order)]
        com = first / m
        second = second - m * np.outer(com, com)
        inertia = np.trace(second) * np.eye(3) - second
        return m, com, inertia

    def write(self, path, comment):
        build = self.build_solid
        track_step.write(path, self.name, comment,
                         [(name, build(loop, t0, t1), rgb)
                          for name, loop, t0, t1, rgb in self.solids],
                         source=SOURCE)

    def build_solid(self, loop, t0, t1):
        """the builder of a solid: in mm, the loop in the plane through
        t0 spanned by the part's axes a and b, extruded along t"""
        e = np.eye(3)
        e1 = tuple(float(c) for c in e[self.axes[0]])
        e2 = tuple(float(c) for c in e[self.axes[1]])
        e3 = track_step.cross(e1, e2)
        sign = e3[self.axes[2]]
        origin = [0.0, 0.0, 0.0]
        origin[self.axes[2]] = float(1000.0 * (t0 if sign > 0 else t1))
        return lambda s, n: track_step.extrude(
            s, n, [segments(loop)], tuple(origin), e1, e2,
            float(1000.0 * (t1 - t0)))

    def on_surface(self, points):
        """whether each point (in the part's coordinates) is on the surface
        of one of its solids, within TOLERANCE"""
        out = np.zeros(len(points), dtype=bool)
        for _, loop, t0, t1, _ in self.solids:
            t = points[:, self.axes[2]]
            near = (t > t0 - TOLERANCE) & (t < t1 + TOLERANCE)
            if not near.any():
                continue
            ab = points[near][:, [self.axes[0], self.axes[1]]]
            on = inside_or_on(loop, ab, TOLERANCE)
            out[np.where(near)[0][on]] = True
        return out


def inside_or_on(loop, points, tolerance):
    """for each point, inside the loop or within tolerance of it"""
    inside = np.zeros(len(points), dtype=bool)
    distance = np.full(len(points), np.inf)
    for i in range(len(loop)):
        a, b = loop[i - 1], loop[i]
        crosses = (a[1] > points[:, 1]) != (b[1] > points[:, 1])
        with np.errstate(divide='ignore', invalid='ignore'):
            x = a[0] + (points[:, 1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
        inside ^= crosses & (x > points[:, 0])
        d = b - a
        s = np.clip(((points - a) @ d) / (d @ d), 0.0, 1.0)
        distance = np.minimum(distance, np.linalg.norm(
            points - (a + s[:, None] * d), axis=1))
    return inside | (distance < tolerance)


# ---------------------------------------------------------------------------
# the parts

PLATE_COLOR = (0.25, 0.25, 0.27)
BUSHING_COLOR = (0.7, 0.7, 0.72)
WHEEL_COLOR = (0.45, 0.47, 0.5)


def track_center(mesh):
    """x of the middle of the tracks (the + side): halfway between the
    planes of the shoes' ends"""
    t = mesh.region([0.5, -0.6, -0.1], [1.6, 0.3, 0.06])
    p = planes(t)
    return 0.5 * (p[0][0] + p[-1][-1]), p[0][0], p[-1][-1]


def bushings(mesh, xc, bushing_area):
    """the centers (y, z) of the bushings of the track at x = xc, in order
    round the loop: the bottom run towards +y"""
    t = mesh.region([xc - 0.06, -3.5, -0.2], [xc + 0.06, 3.0, 1.3])
    loops = [l for l in section(t, xc)
             if abs(area(l) - bushing_area) < 0.02 * bushing_area]
    centers = np.array([fit_circle(l)[0] for l in loops])
    middle = centers.mean(0)
    angle = np.arctan2(centers[:, 1] - middle[1], centers[:, 0] - middle[0])
    centers = centers[np.argsort(angle)]
    gaps = np.linalg.norm(np.roll(centers, -1, 0) - centers, axis=1)
    if np.abs(gaps - PITCH).max() > 5e-4:
        raise RuntimeError('the bushings at x = %g are not %g apart: %g..%g'
                           % (xc, PITCH, gaps.min(), gaps.max()))
    return centers


def link_part(mesh, xc, centers):
    """the link whose bushing is the bottom run's nearest y = -0.26: its
    body (the shoe and the link plates welded to it) a solid of the mesh's
    faces, the bushing a prism; and the bushing's center (y, z)

    The links touch where their plates meet, so the faces of the body are
    told from the next links' by the cross section just inside each face
    (INSET along its inward normal): the face is the link's if the loop
    there is the one round the link's shoe."""
    bottom = centers[centers[:, 1] < 0.2]
    yb, zb = bottom[np.argmin(np.abs(bottom[:, 0] + 0.26))]
    t = mesh.region([xc - 0.4, yb - 0.4, zb - 0.2],
                    [xc + 0.4, yb + 0.6, 0.1329])
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    n /= np.linalg.norm(n, axis=1)[:, None]
    # a point just inside each face, clear of the planes between the slabs
    # (where a cross section is degenerate)
    breaks = np.array([b[0] for b in planes(t)] + [b[-1] for b in planes(t)])
    inner = np.zeros((len(t), 3))
    for i, tri in enumerate(t):
        best, clearance = None, -1.0
        for w in ((1, 1, 1), (4, 1, 1), (1, 4, 1), (1, 1, 4), (2, 2, 1),
                  (2, 1, 2), (1, 2, 2)):
            q = (np.array(w) @ tri) / sum(w) - INSET * n[i]
            c = np.abs(breaks - q[0]).min()
            if c > clearance:
                best, clearance = q, c
            if c > 2.0 * SECTION_STEP:
                break
        inner[i] = best
    # the shoe's plate, halfway to the next bushing; the bushing at the
    # link's pin, through the holes of its inner plates
    shoe_point = np.array([yb + 0.5 * PITCH, zb - 0.075])
    pin = np.array([yb + PITCH, zb])
    body, bushing = [], None
    keys = np.round(inner[:, 0] / SECTION_STEP).astype(int)
    for key in sorted(set(keys)):
        members = np.where(keys == key)[0]
        loops = section(t, key * SECTION_STEP)
        own = [l for l in loops if contains(l, shoe_point)]
        sleeve = [l for l in loops if abs(area(l)) < 0.005 and
                  contains(l, pin)]
        for i in members:
            q = inner[i, 1:]
            if own and contains(own[0], q):
                body.append(t[i])
            elif sleeve and contains(sleeve[0], q):
                if bushing is None:
                    bushing = sleeve[0]
    if bushing is None:
        raise RuntimeError('no bushing at y = %g, z = %g' % tuple(pin))
    # from the mesh's (x, y, z) to the link's (x along the track, y along
    # the bushing, z out of the loop): a rotation, so the faces keep their
    # winding
    def local(p):
        return np.stack([p[..., 1] - yb, p[..., 0] - xc, zb - p[..., 2]],
                        axis=-1)
    link = Faceted('excavator_track_link', local(np.array(body)))
    # the bushing's ends: those of the slab it is in
    _, r = fit_circle(bushing)
    b = t[np.all(np.linalg.norm(t[:, :, 1:] - pin, axis=2) < r + TOLERANCE,
                 axis=1)]
    v0, v1 = b[:, :, 0].min() - xc, b[:, :, 0].max() - xc
    sleeve = np.c_[bushing[:, 0] - yb, zb - bushing[:, 1]][::-1]
    link.prism = Part('bushing', (0, 2, 1))
    link.prism.add('bushing', sleeve, v0, v1, BUSHING_COLOR)
    return link, (yb, zb)


class Faceted:
    """a solid bounded by the triangles of a mesh, welded at their vertices
    and drawn as planar faces, each the coplanar triangles that share
    edges; with a prism (Part) beside it"""

    def __init__(self, name, triangles):
        self.name = name
        self.triangles = triangles
        self.prism = None
        keys = {}
        self.points = []
        index = np.zeros(triangles.shape[:2], dtype=int)
        for i, tri in enumerate(triangles):
            for k, p in enumerate(tri):
                key = tuple(np.round(p / 1e-7).astype(np.int64))
                if key not in keys:
                    keys[key] = len(self.points)
                    self.points.append(p)
                index[i, k] = keys[key]
        self.points = np.array(self.points)
        self.index = self.capped(index)
        edges = collections.Counter()
        for a, b, c in self.index:
            for e in ((a, b), (b, c), (c, a)):
                edges[e] += 1
        bad = [e for e in edges if edges[e] != 1 or
               edges.get((e[1], e[0]), 0) != 1]
        if bad:
            raise RuntimeError('%s: %d edges not shared by two triangles '
                               'wound opposite ways' % (name, len(bad)))

    def capped(self, index):
        """the triangles with the planar holes of the surface (where the
        bushing passes through the plates) closed by fans of triangles"""
        directed = set()
        for a, b, c in index:
            directed.update(((a, b), (b, c), (c, a)))
        following = {}
        for a, b in directed:
            if (b, a) not in directed:
                following[a] = b
        caps = []
        while following:
            start = next(iter(following))
            loop = [start]
            while following[loop[-1]] != start:
                loop.append(following[loop[-1]])
                if len(loop) > len(self.points):
                    raise RuntimeError('%s: an open edge that does not '
                                       'close' % self.name)
            for k in loop:
                del following[k]
            p = self.points[loop]
            normal = np.cross(p - p.mean(0), np.roll(p, -1, 0) - p.mean(0)).sum(0)
            normal /= np.linalg.norm(normal)
            if np.abs((p - p.mean(0)) @ normal).max() > 1e-7:
                raise RuntimeError('%s: a hole that is not planar' % self.name)
            # the hole runs the way its neighbors' edges do, so the cap
            # runs the other way
            center = len(self.points)
            self.points = np.vstack([self.points, p.mean(0)])
            for i in range(len(loop)):
                caps.append((center, loop[(i + 1) % len(loop)], loop[i]))
        return np.vstack([index, np.array(caps, dtype=int)]) if caps \
            else index

    def faces(self):
        """the planar faces: (normal, loops of point indices), the outer
        loop first"""
        normals = np.cross(self.points[self.index[:, 1]] -
                           self.points[self.index[:, 0]],
                           self.points[self.index[:, 2]] -
                           self.points[self.index[:, 0]])
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        by_edge = {}
        for i, (a, b, c) in enumerate(self.index):
            for e in ((a, b), (b, c), (c, a)):
                by_edge[e] = i
        group = [-1] * len(self.index)
        out = []
        for seed in range(len(self.index)):
            if group[seed] >= 0:
                continue
            members, stack = [], [seed]
            group[seed] = seed
            while stack:
                i = stack.pop()
                members.append(i)
                a, b, c = self.index[i]
                for e in ((a, b), (b, c), (c, a)):
                    j = by_edge[(e[1], e[0])]
                    if group[j] < 0 and normals[i] @ normals[j] > 1 - 1e-9:
                        group[j] = seed
                        stack.append(j)
            directed = set()
            for i in members:
                a, b, c = self.index[i]
                directed.update(((a, b), (b, c), (c, a)))
            boundary = [e for e in directed if (e[1], e[0]) not in directed]
            following = collections.defaultdict(list)
            for a, b in boundary:
                following[a].append(b)
            loops, used = [], set()
            for a, b in boundary:
                if (a, b) in used:
                    continue
                loop = [a]
                used.add((a, b))
                previous, current = a, b
                while current != loop[0]:
                    loop.append(current)
                    options = [c for c in following[current]
                               if (current, c) not in used]
                    nxt = options[0]
                    if len(options) > 1:
                        # sharpest left turn, about the face's normal
                        d0 = self.points[current] - self.points[previous]

                        def turn(c):
                            d1 = self.points[c] - self.points[current]
                            return math.atan2(
                                np.cross(d0, d1) @ normals[seed], d0 @ d1)
                        nxt = max(options, key=turn)
                    used.add((current, nxt))
                    previous, current = current, nxt
                loops.append(loop)

            def loop_area(loop):
                p = self.points[loop]
                return 0.5 * np.cross(p - p[0], np.roll(p, -1, 0) - p[0]).sum(
                    0) @ normals[seed]
            loops.sort(key=lambda l: -loop_area(l))
            out.append((normals[seed], loops))
        return out

    def on_surface(self, points):
        """whether each point (in the part's coordinates) is the center of
        one of its triangles or on the surface of its prism"""
        from scipy.spatial import cKDTree
        tree = cKDTree(self.triangles.mean(1))
        distance, _ = tree.query(points)
        out = distance < TOLERANCE
        if self.prism is not None:
            out |= self.prism.on_surface(points)
        return out

    def mass(self):
        """mass, center of mass and inertia about it, of steel, and of the
        prism beside it"""
        p0, p1, p2 = (self.points[self.index[:, k]] for k in range(3))
        volume = np.einsum('ij,ij->i', p0, np.cross(p1, p2)) / 6.0
        m = STEEL_DENSITY * volume.sum()
        first = STEEL_DENSITY * (volume[:, None] * (p0 + p1 + p2) / 4.0).sum(0)
        # second moments of each tetrahedron from the origin
        s = p0 + p1 + p2
        second = np.zeros((3, 3))
        for j in range(3):
            for k in range(3):
                second[j, k] = STEEL_DENSITY * (volume / 20.0 * (
                    s[:, j] * s[:, k] + p0[:, j] * p0[:, k] +
                    p1[:, j] * p1[:, k] + p2[:, j] * p2[:, k])).sum()
        if self.prism is not None:
            for _, loop, t0, t1, _ in self.prism.solids:
                mi, fi, si = prism_mass(loop, t0, t1)
                order = np.argsort(self.prism.axes)
                m += mi
                first += fi[order]
                second += si[np.ix_(order, order)]
        com = first / m
        second = second - m * np.outer(com, com)
        return m, com, np.trace(second) * np.eye(3) - second

    def write(self, path, comment, rgb):
        def build(step, name):
            vertices = {}

            def vertex(i):
                if i not in vertices:
                    vertices[i] = step.add("VERTEX_POINT('',#%d)" % step.point(
                        mm(self.points[i])))
                return vertices[i]
            edges = {}

            def edge(a, b):
                # each edge once, from its lower vertex; the oriented edge
                # along it from a to b
                key = (min(a, b), max(a, b))
                if key not in edges:
                    p = np.array(mm(self.points[key[0]]))
                    q = np.array(mm(self.points[key[1]]))
                    line = step.add("LINE('',#%d,#%d)" % (
                        step.point(mm(self.points[key[0]])),
                        step.add("VECTOR('',#%d,%s)" % (
                            step.direction(tuple(float(c) for c in q - p)),
                            step.num(float(np.linalg.norm(q - p)))))))
                    edges[key] = step.add("EDGE_CURVE('',#%d,#%d,#%d,.T.)" % (
                        vertex(key[0]), vertex(key[1]), line))
                return step.add("ORIENTED_EDGE('',*,*,#%d,%s)" % (
                    edges[key], '.T.' if a == key[0] else '.F.'))
            faces = []
            for normal, loops in self.faces():
                bounds = []
                for k, loop in enumerate(loops):
                    oriented = [edge(loop[i], loop[(i + 1) % len(loop)])
                                for i in range(len(loop))]
                    edge_loop = step.add("EDGE_LOOP('',(%s))" % ','.join(
                        '#%d' % e for e in oriented))
                    bounds.append(step.add("%s('',#%d,.T.)" % (
                        'FACE_OUTER_BOUND' if k == 0 else 'FACE_BOUND',
                        edge_loop)))
                a, b = self.points[loops[0][0]], self.points[loops[0][1]]
                plane = step.add("PLANE('',#%d)" % step.placement(
                    mm(a), tuple(float(c) for c in normal),
                    tuple(float(c) for c in b - a)))
                faces.append(step.add("ADVANCED_FACE('',(%s),#%d,.T.)" % (
                    ','.join('#%d' % x for x in bounds), plane)))
            shell = step.add("CLOSED_SHELL('',(%s))" % ','.join(
                '#%d' % f for f in faces))
            return step.add("MANIFOLD_SOLID_BREP('%s',#%d)" % (name, shell))
        solids = [('body', build, rgb)]
        if self.prism is not None:
            part = self.prism
            for name, loop, t0, t1, color in part.solids:
                solids.append((name, part.build_solid(loop, t0, t1), color))
        track_step.write(path, self.name, comment, solids, script=SOURCE)


def wheel_part(mesh, name, xc, center, radius):
    """the solids of a wheel about center (y, z) within radius of it, the
    slabs of the same cross section merged"""
    c = np.array(center)
    t = mesh.region([xc - 0.25, c[0] - radius, c[1] - radius],
                    [xc + 0.25, c[0] + radius, c[1] + radius])
    r = np.linalg.norm(t[:, :, 1:] - c, axis=2).max(1)
    t = t[r < radius]
    bounds = planes(t)
    part = Part(name, (0, 1, 2))  # (x, y) = the mesh's (y, z), along z
    previous = None
    for lo, hi in zip(bounds[:-1], bounds[1:]):
        x0, x1 = lo[-1], hi[0]
        # the wheel's solids are discs, around its center
        loops = [l for l in section(t, 0.5 * (x0 + x1))
                 if area(l) > 0 and contains(l, c)]
        if not loops:
            continue
        loop = max(loops, key=area)
        local = loop - c
        if (previous is not None and abs(previous[3] - (x0 - xc)) < 1e-6 and
                abs(area(previous[1]) - area(local)) < 1e-6):
            part.solids[-1] = (previous[0], previous[1], previous[2],
                               x1 - xc, WHEEL_COLOR)
        else:
            part.add('%s %d' % (name.split('_')[-1], len(part.solids) + 1),
                     local, x0 - xc, x1 - xc, WHEEL_COLOR)
        previous = part.solids[-1][:4]
    return part


def wheels(mesh, xc, bushing_area):
    """the wheels of the track at x = xc, from its cross section there:
    {kind: [center (y, z)]}, and the seats' centers of the sprocket"""
    t = mesh.region([xc - 0.06, -3.5, -0.2], [xc + 0.06, 3.0, 1.3])
    found = collections.defaultdict(list)
    sprocket = None
    for loop in section(t, xc):
        a = abs(area(loop))
        if a < 2.0 * bushing_area:
            continue
        center, r = fit_circle(loop)
        spread = np.ptp(np.linalg.norm(loop - center, axis=1))
        if spread > 0.01:
            sprocket = loop
        elif r > 0.2:
            found['idler'].append(center)
        elif r > 0.09:
            found['track_roller'].append(center)
        elif r > 0.05:
            found['carrier_roller'].append(center)
    # the sprocket: its center that of its seats, the arcs nearest it
    middle = sprocket.mean(0)
    r = np.linalg.norm(sprocket - middle, axis=1)
    order = np.argsort(np.arctan2(sprocket[:, 1] - middle[1],
                                  sprocket[:, 0] - middle[0]))
    q, r = sprocket[order], r[order]
    low = np.where(r < r.min() + 0.25 * np.ptp(r))[0]
    groups = [g for g in np.split(low, np.where(np.diff(low) > 1)[0] + 1)
              if len(g) >= 5]
    seats = np.array([fit_circle(q[g])[0] for g in groups])
    center, _ = fit_circle(seats)
    found['sprocket'].append(center)
    for kind in found:
        found[kind].sort(key=lambda c: c[0])
    return found, seats - center


# ---------------------------------------------------------------------------
# the frame: the undercarriage without the parts

def write_frame(mesh, placed, path):
    """the mesh without the faces whose triangles all lie on a part placed
    (part, origin, rotation: columns the part's axes in the mesh)"""
    centroids = mesh.triangles.mean(1)
    on = np.zeros(len(centroids), dtype=bool)
    for part, origin, rotation, extent in placed:
        near = np.all(np.abs(centroids - origin) < extent, axis=1)
        if not near.any():
            continue
        local = (centroids[near] - origin) @ rotation
        on[np.where(near)[0][part.on_surface(local)]] = True
    drop = set(mesh.faces[on]) - set(mesh.faces[~on])
    # the faces kept, their vertices, texture coordinates and normals
    # numbered again
    kinds = {'v': [], 'vt': [], 'vn': []}
    for line in mesh.lines:
        tag = line.split(' ', 1)[0]
        if tag in kinds:
            kinds[tag].append(line)
    used = {k: {} for k in kinds}
    body = []
    for i, line in enumerate(mesh.lines):
        if line.startswith('f '):
            if i in drop:
                continue
            refs = []
            for token in line.split()[1:]:
                parts = token.split('/')
                out = []
                for kind, p in zip(('v', 'vt', 'vn'), parts):
                    if p == '':
                        out.append('')
                        continue
                    table = used[kind]
                    k = int(p) - 1
                    if k not in table:
                        table[k] = len(table)
                    out.append(str(table[k] + 1))
                refs.append('/'.join(out))
            body.append('f ' + ' '.join(refs))
        elif line.startswith(('mtllib', 'usemtl', 'o ', 'g ', 's ')):
            body.append(line)
    head = ['# The undercarriage of examples/excavator/undercarriage.obj '
            'without its tracks, sprockets, idlers and rollers. Written by '
            + SOURCE + '.']
    for kind in ('v', 'vt', 'vn'):
        for k, _ in sorted(used[kind].items(), key=lambda e: e[1]):
            head.append(kinds[kind][k])
    with open(path, 'w') as f:
        f.write('\n'.join(head + body) + '\n')
    return len(drop), len(set(mesh.faces))


# ---------------------------------------------------------------------------

def link_frames(centers, xc):
    """the origin (mesh) and rotation (columns its axes in the mesh) of
    each link of a track at x = xc: the origin at its bushing center, x
    towards the next one, y along x of the mesh, z out of the loop"""
    frames = []
    for i in range(len(centers)):
        a, b = centers[i], centers[(i + 1) % len(centers)]
        d = (b - a) / np.linalg.norm(b - a)
        x = np.array([0.0, d[0], d[1]])
        y = np.array([1.0, 0.0, 0.0])
        frames.append((np.array([xc, a[0], a[1]]),
                       np.column_stack([x, y, np.cross(x, y)])))
    return frames


def wheel_rotation(phase):
    """columns the axes of a wheel turned by phase about its axle (x of the
    mesh): x forward and y up when phase is 0"""
    c, s = math.cos(phase), math.sin(phase)
    return np.column_stack([[0.0, c, s], [0.0, -s, c], [1.0, 0.0, 0.0]])


def sprocket_phase(seats, reference, teeth):
    """the turn of a sprocket whose seats (from its center) are seats from
    one whose seats are reference: the turn of its seats from the nearest
    of the reference's, within half a tooth"""
    pitch = 2.0 * math.pi / teeth
    a = np.arctan2(seats[:, 1], seats[:, 0])
    b = math.atan2(reference[0, 1], reference[0, 0])
    d = np.mod(a - b + 0.5 * pitch, pitch) - 0.5 * pitch
    if np.ptp(d) > 1e-3:
        raise RuntimeError('the seats of the sprockets differ')
    return float(np.mean(d))


def extract(out):
    """the parts and where the mesh has them"""
    mesh = Mesh(os.path.join(out, 'undercarriage.obj'))
    xc, _, _ = track_center(mesh)
    bushing_t = mesh.region([xc - 0.06, -0.4, 0.05], [xc + 0.06, -0.1, 0.14])
    # the smallest loop there, the bushing's
    bushing_area = min(abs(area(l)) for l in section(bushing_t, xc))
    sides = {}
    for side in (1, -1):
        centers = bushings(mesh, side * xc, bushing_area)
        found, seats = wheels(mesh, side * xc, bushing_area)
        sides[side] = {'centers': centers, 'wheels': found, 'seats': seats}
    link, _ = link_part(mesh, xc, sides[1]['centers'])
    w = sides[1]['wheels']
    parts = {
        'sprocket': wheel_part(mesh, 'excavator_sprocket', xc,
                               w['sprocket'][0], 0.43),
        'idler': wheel_part(mesh, 'excavator_idler', xc, w['idler'][0], 0.38),
        'track_roller': wheel_part(mesh, 'excavator_track_roller', xc,
                                   w['track_roller'][0], 0.13),
        'carrier_roller': wheel_part(mesh, 'excavator_carrier_roller', xc,
                                     w['carrier_roller'][0], 0.08),
    }
    # the teeth from the spacing of the seats found
    a = np.sort(np.arctan2(sides[1]['seats'][:, 1], sides[1]['seats'][:, 0]))
    teeth = int(round(2.0 * math.pi / np.median(np.diff(a))))
    for side in (1, -1):
        sides[side]['sprocket_phase'] = sprocket_phase(
            sides[side]['seats'], sides[1]['seats'], teeth)
    return mesh, xc, sides, link, parts, teeth


# ---------------------------------------------------------------------------
# the model

# the masses, from published data, publicly available, for the Caterpillar
# 323, a machine of the mesh's main dimensions (3650 mm from the sprocket to
# the idler, a gauge of 2380 mm, 600 mm shoes, a tail swing radius of
# 2830 mm; the mesh was drawn independently): "Major Component Weights" in
# Caterpillar, Technical Specifications for 323 Hydraulic Excavator,
# AEXQ3361-01 (2022), an operating weight of 24 100 kg with the 5.4 t
# counterweight; a copy is at
# https://www.avesco-cat.com/fileadmin/unsorted/323_Specification.pdf. excavator.json's (30.6 t in all, its upper
# structure 13.2 t with its center of mass 0.7 m behind the swing axis)
# tip the machine forward on the tracks
UNDERCARRIAGE_MASS = 7550.0   # kg: the base frame with its rollers, 4470,
                              # and the track shoes, 3080
LINK_MASS = 3080.0 / 92.0     # kg, of a link with its shoe
# kg, of the wheels (their solids at the density of steel, the mesh's
# prisms, are 2 to 5 times heavier), from publicly available sources:
# - track roller 40 kg (88 lb), carrier roller 17 kg (38 lb), front idler
#   109 kg (240 lb): Caterpillar,
#   320D L and 320D LN Excavators (PCX) Disassembly and Assembly, RENR8614
#   (SMCS 4180, 4154); excerpt at https://www.slideshare.net/uddksemd8ud/
#   caterpillar-cat-320dln-excavator-prefix-pcx-service-repair-manual-
#   pcx00001-and-uppdf
# - sprocket 40 kg: VXB Bearings 23 tooth drive sprocket for SH100 to SH265
#   (1150-6425), https://vxb.com/products/heavy-duty-23-tooth-drive-sprocket-
#   with-15-mountin (none is published for the 320 or 323)
# the frame gets the rest of the 4470
WHEEL_MASSES = {'excavator_sprocket': 40.0, 'excavator_idler': 109.0,
                'excavator_track_roller': 40.0,
                'excavator_carrier_roller': 17.0}
# the upper structure without the counterweight: the base machine, 16 400,
# less the undercarriage's 4470 and the counterweight, with 310 of fuel and
# operator
UPPER_MASS = 6840.0
COUNTERWEIGHT_MASS = 5400.0
# the counterweight's block in the cabin's mesh (z up): its center and size
COUNTERWEIGHT_CENTER = [0.0, 2.4, 0.67]
COUNTERWEIGHT_SIZE = [3.0, 0.8, 1.08]
# the front, each group of bodies to its weight, shared as excavator.json
# shares it: the boom with the stick cylinder, the stick with the bucket
# cylinder and linkage, the boom cylinders, the bucket (the operating weight
# less the rest)
FRONT_MASSES = [
    (('boom', 'stick_cylinder', 'stick_rod'), 2010.0),
    (('stick', 'bucket_cylinder', 'bucket_rod', 'bucket_linkage_1',
      'bucket_linkage_2'), 990.0),
    (('boom_cylinder_1', 'boom_rod_1', 'boom_cylinder_2', 'boom_rod_2'),
     340.0),
    (('bucket',), 970.0)]
# the frame's distribution of mass: that of the convex hull of
# undercarriage_frame.obj (its faces are not a closed surface), from shift's
# density helper at unit density: volume (m^3), center of mass and the
# moments of inertia about it (m^5), in the mesh's coordinates
FRAME_HULL_VOLUME = 9.055186688
FRAME_HULL_CENTER = [0.0, -0.4140770308, 0.5287844658]
FRAME_HULL_INERTIA = [10.79737294, 6.420310723, 16.16206048, 0.0,
                      -0.0544961182, 0.0]  # Ixx Iyy Izz Ixy Iyz Izx
# from the mesh's coordinates (z up, the idler towards -y) to the model's
# (x forward, y to the right, z down): the rows
MESH_TO_MODEL = [[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]
GROUND_GAP = 5e-4             # m, of the grousers' tips off the ground
# the sand, a terrain of type scm (m): its extent, which the drive stays on
# (the machine's footprint, 4.7 x 3 m, turning on the spot), and the spacing
# of its nodes. Its settling lets the machine down into it gently from
# GROUND_GAP: on dry sand the shoes sink about 107 mm
SAND_X = [-3.0, 7.5]
SAND_Y = [-4.5, 3.0]
SAND_SPACING = 0.05
SAND_SETTLE_TIME = 1.0  # s, as the drive settles
# the rigid ground that holds the sand as a region, a terrain of type basic
# drawn with the same texture: a square of GROUND_SIZE (m) about the sand,
# its elevation map excavator_ground.tiff (written by this script)
# GROUND_SPACING apart. The sand's undisturbed surface is the ground's. The
# ground rolls gently, in swells: amplitude (m), wavelength (m), heading
# (deg) and phase (rad), flat under the machine as it starts (x and y of the
# model, the footprint of the tracks) and all of it GROUND_BLEND (m) beyond.
# Off the sand, the shoes are on track_soil's ground plane, z = 0
GROUND_SIZE = 60.0
GROUND_SPACING = 0.5
GROUND_SWELLS = [(0.04, 14.0, 20.0, 0.7),
                 (0.03, 9.0, -50.0, 2.3),
                 (0.015, 5.0, 75.0, 4.1)]
GROUND_START_X = (-2.0, 2.8)
GROUND_START_HALF_Y = 1.6
GROUND_BLEND = 2.0
TREAD_GAP = 5e-4              # m, of the bushings off the wheels' treads,
                              # and of the rails off the rollers
PIN_CLEARANCE = 2.5e-3        # m, of a bushing in a seat to the pins
RAIL_INNER = 0.0688           # m, of each rail from the middle of the link:
                              # clear of the treads of the sprocket and idler
# m, the semi-axis along the axle of the treads of the sprocket and the
# idler, flat ellipsoids under the bushings. Spheres of their radius would
# reach the rails, which cut across the wheel along the chord between two
# bushings, 13 mm inside the pitch circle; at the rails' inner edge these are
# 11% smaller, 20 mm clear of them. Cylinders would do as well, but their
# contact with the bushings' spheres is a search (GJK and EPA) whose depth
# varies with its tolerance, and the integrator takes steps 10 times shorter
# through it; the contact of an ellipsoid and a sphere is exact
TREAD_SEMI_AXIS = 0.15
RAIL_OUTER = 0.1100           # m: 1 mm inside the flanges of the idler and
                              # the track rollers
# the spheres round each flange of the idler and of the track rollers, each
# the circle inscribed in the flange's cross section. A cylinder as a flange
# would meet the rails' boxes through a search (GJK and EPA) whose depth
# varies with its tolerance: the integrator then takes steps 8 times shorter
# while the tracks are pushed sideways in a turn. The contact of a sphere and
# a box is exact
IDLER_FLANGE_SPHERES = 24     # 91 mm apart, under every link round it
ROLLER_FLANGE_SPHERES = 12    # 30 degrees apart: two or more beside the rails
# the drive: the sprockets' speed (rad/s) against time, both forward, then
# turning on the spot to the left (the left track back, the right forward),
# then forward again: control points [t, left, right]
FORWARD = 1.5
TURN = 1.2
DRIVE = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, FORWARD, FORWARD),
         (5.0, FORWARD, FORWARD), (6.0, -TURN, TURN), (9.0, -TURN, TURN),
         (10.0, FORWARD, FORWARD), (13.0, FORWARD, FORWARD),
         (14.0, 0.0, 0.0)]
TIME_SPAN = 15.0
# the track adjuster: each idler on a yoke that slides away from the
# sprocket, pushed by a preloaded spring. With the idler fixed the track has
# no tension, and the links ahead of the sprocket, pushed into the ground,
# fold back and forth. The spring pushes with IDLER_SPRING_PRELOAD (N), a
# chain tension of half of it in each run, above the 60 kN or so a track can
# pull on dry sand (0.4 of the half of 30.6 t on it)
IDLER_SPRING_PRELOAD = 1.5e5
IDLER_SPRING_STIFFNESS = 5.0e5  # N/m
IDLER_SPRING_DAMPING = 2.0e4    # N s/m
IDLER_SPRING_LENGTH = 0.5       # m, from its seat on the frame to the yoke
IDLER_YOKE_MASS = 80.0          # kg
# the forces (N, along the extension) with which the cylinders of
# excavator.json hold the arm still on the undercarriage fixed, from its
# "hold" (Context::HoldInitialStates). On the tracks the undercarriage is
# free and the ground carries no load before the simulation starts, so the
# hold has no equilibrium to find; the cylinders start with the pressures
# that make these forces instead
HOLD_FORCES = {'boom_cylinder_1': 195253.745039,
               'boom_cylinder_2': 195253.745039,
               'stick_cylinder': -122579.113135,
               'bucket_cylinder': -21387.808444}


def read_json(path):
    """a JSON file with // comments and trailing commas, as shift reads it"""
    text = open(path).read()
    out, i, in_string = [], 0, False
    while i < len(text):
        c = text[i]
        if in_string:
            out.append(c)
            if c == '\\':
                out.append(text[i + 1])
                i += 1
            elif c == '"':
                in_string = False
        elif c == '"':
            in_string = True
            out.append(c)
        elif text.startswith('//', i):
            while i < len(text) and text[i] != '\n':
                i += 1
            continue
        else:
            out.append(c)
        i += 1
    return json.loads(re.sub(r',(\s*[}\]])', r'\1', ''.join(out)))


def number(x):
    x = float(x)
    if abs(x) < 1e-12:
        return 0.0
    return float('%.10g' % x)


def dump(value, indent=0):
    """JSON text of value, a key '//' of a dictionary written as // comment
    lines (shift reads them), short lists of numbers on one line"""
    pad = '  ' * (indent + 1)
    if isinstance(value, dict):
        lines = []
        items = list(value.items())
        for k, (key, v) in enumerate(items):
            if key == '//':
                for line in wrap(v, 76 - len(pad)):
                    lines.append(pad + '// ' + line)
                continue
            last = all(key2 == '//' for key2, _ in items[k + 1:])
            lines.append(pad + json.dumps(key) + ': ' + dump(v, indent + 1) +
                         ('' if last else ','))
        return '{\n' + '\n'.join(lines) + '\n' + '  ' * indent + '}'
    if isinstance(value, list):
        flat = json.dumps(value)
        if all(not isinstance(v, (dict, list)) for v in value) and \
                len(flat) < 70:
            return flat
        if all(isinstance(v, list) and
               all(not isinstance(x, (dict, list)) for x in v)
               for v in value) and len(flat) < 70:
            return flat
        return '[\n' + ',\n'.join(pad + dump(v, indent + 1)
                                  for v in value) + '\n' + '  ' * indent + ']'
    if isinstance(value, float):
        return json.dumps(number(value))
    return json.dumps(value)


def wrap(text, width):
    words, lines, line = text.split(), [], ''
    for w in words:
        if line and len(line) + 1 + len(w) > width:
            lines.append(line)
            line = w
        else:
            line = (line + ' ' + w) if line else w
    if line:
        lines.append(line)
    return lines


def inertia(mass, matrix):
    return {'mass': number(mass), 'Ixx': number(matrix[0][0]),
            'Iyy': number(matrix[1][1]), 'Izz': number(matrix[2][2]),
            'Ixy': number(matrix[0][1]), 'Iyz': number(matrix[1][2]),
            'Izx': number(matrix[2][0])}


def rows(matrix):
    return [[number(x) for x in row] for row in np.asarray(matrix)]


def vector(v):
    return [number(x) for x in v]


def scaled(inertia_, mass):
    """an inertia of the body's mass made mass, its distribution kept"""
    k = mass / inertia_['mass']
    return {key: number(value * k) for key, value in inertia_.items()}


def holding(cylinder):
    """a hydraulic cylinder of excavator.json that starts with the
    pressures of its force in HOLD_FORCES, as its hold would make them: the
    pressure on one side, the tank's on the other"""
    if cylinder.get('type') != 'hydraulic_cylinder' or \
            cylinder['name'] not in HOLD_FORCES:
        return cylinder
    out = dict(cylinder)
    del out['hold']
    head = 0.25 * math.pi * cylinder['bore_diameter'] ** 2
    rod = head - 0.25 * math.pi * cylinder['rod_diameter'] ** 2
    tank = cylinder['tank_pressure']
    force = cylinder['direction'] * HOLD_FORCES[cylinder['name']]
    if force >= tank * (head - rod):
        pressures = [(force + tank * rod) / head, tank]
    else:
        pressures = [tank, (tank * head - force) / rod]
    out['initial_pressures'] = [number(p) for p in pressures]
    out['//'] = ('the pressures that hold the arm still, as hold does with '
                 'the undercarriage fixed')
    return out


SAND_TEXTURE = ('qrc:/textures/dry_sand.jpg',
                'qrc:/textures/dry_sand_normal.jpg')


def ground(out, sand):
    """the rigid ground, holding the sand as a region: writes its elevation
    map, excavator_ground.tiff, and returns its frame and terrain"""
    from PIL import Image
    s = GROUND_SPACING
    middle = [0.5 * (SAND_X[0] + SAND_X[1]), 0.5 * (SAND_Y[0] + SAND_Y[1])]
    x0 = middle[0] - 0.5 * GROUND_SIZE
    y0 = middle[1] - 0.5 * GROUND_SIZE
    n = int(round(GROUND_SIZE / s)) + 1
    x, y = np.meshgrid(x0 + s * np.arange(n), y0 + s * np.arange(n))
    height = np.zeros_like(x)  # up
    for amplitude, wavelength, heading, phase in GROUND_SWELLS:
        along = x * math.cos(math.radians(heading)) + \
            y * math.sin(math.radians(heading))
        height += amplitude * np.sin(2.0 * math.pi * along / wavelength +
                                     phase)
    # flat under the start, by the distance outside the tracks' footprint
    dx = np.maximum(np.maximum(GROUND_START_X[0] - x,
                               x - GROUND_START_X[1]), 0.0)
    dy = np.maximum(np.abs(y) - GROUND_START_HALF_Y, 0.0)
    blend = np.clip(np.hypot(dx, dy) / GROUND_BLEND, 0.0, 1.0)
    height *= blend * blend * (3.0 - 2.0 * blend)
    # a sample is the z of the surface (+z down), a row along x
    Image.fromarray((-height).astype(np.float32), mode='F').save(
        os.path.join(out, 'excavator_ground.tiff'))
    frame = {'name': 'ground', 'T': vector([x0, y0, 0.0])}
    # the region in the ground's frame
    sand = dict(sand, x=vector([SAND_X[0] - x0, SAND_X[1] - x0]),
                y=vector([SAND_Y[0] - y0, SAND_Y[1] - y0]))
    terrain = {
        '//': 'rigid ground, %g m square, rolling gently, with the sand '
              'where the machine drives' % GROUND_SIZE,
        'type': 'basic', 'name': 'ground', 'parent': [['fixed'], 'ground'],
        'filename': 'excavator_ground.tiff', 'x_scale': s, 'y_scale': s,
        'texture_filename': SAND_TEXTURE[0],
        'normal_map_filename': SAND_TEXTURE[1], 'texture_scale': 0.5,
        'regions': [sand]}
    return frame, terrain


def build_model(out, mesh, xc, sides, link, parts, teeth):
    original = read_json(os.path.join(out, 'excavator.json'))
    upper = original['assemblies'][0]

    # the link's shoe, for the soil: its middle between its ends along the
    # track, the plane of the grousers' tips, and the plate's underside
    points = link.points
    shoe = points[np.abs(points[:, 1]).max() - np.abs(points[:, 1]) < 1e-6]
    tips = points[:, 2].max()
    levels = collections.Counter(np.round(shoe[:, 2], 4))
    plate = max(w for w in levels if w < tips - 0.01 and
                np.ptp(shoe[np.abs(shoe[:, 2] - w) < 5e-5][:, 0]) > 0.1)
    shoe_center = [0.5 * (shoe[:, 0].min() + shoe[:, 0].max()), 0.0, tips]
    shoe_width = 2.0 * np.abs(points[:, 1]).max()
    rail = points[:, 2].min()  # the link plates' faces towards the wheels

    # the wheels' pitch circles: the sprocket's seats, the bushings round
    # the idler
    seat_radius = float(np.mean(np.linalg.norm(sides[1]['seats'], axis=1)))
    idler = np.array(sides[1]['wheels']['idler'][0])
    r = np.linalg.norm(sides[1]['centers'] - idler, axis=1)
    idler_pitch = float(np.median(r[r < 0.42]))
    bushing_radius = fit_circle(link.prism.solids[0][1])[1]

    rotation = np.array(MESH_TO_MODEL)
    # the lowest of the grousers' tips, in the mesh, on the ground plane
    z_low = min((origin + rot @ np.array([0.0, 0.0, tips]))[2]
                for side in (1, -1)
                for origin, rot in link_frames(sides[side]['centers'],
                                               side * xc))
    translation = np.array([0.0, 0.0, z_low - GROUND_GAP])

    def to_model(p):
        return rotation @ p + translation

    def wheel_frame(name, center, side):
        return {'name': name,
                'T': vector([side * xc, center[0], center[1]]),
                'R': rows(wheel_rotation(0.0))}

    def mass_of(part):
        m, com, matrix = part.mass()
        return vector(com), inertia(m, matrix)

    def sphere(radius, at):
        return {'type': 'sphere', 'radius': number(radius),
                'material': 'steel', 'pose': {'T': vector(at)}}

    def tread(radius):
        return {'type': 'ellipsoid', 'material': 'steel',
                'semi_axes': vector([radius, radius, TREAD_SEMI_AXIS]),
                'pose': {'T': [0.0, 0.0, 0.0]}}

    def flanges(part, count):
        """the spheres round the flanges of a wheel of a hub between two
        flanges, each touching the flange's face towards the hub and its
        rim, and the hub's tread if the flange is no thicker than tall"""
        solids = solid_extent(part)
        hub = next(e for e in solids if e[0] < 0.0 < e[1])
        out = []
        for t0, t1, radius in solids:
            if t0 < 0.0 < t1:
                continue
            r = 0.5 * min(t1 - t0, radius - hub[2])
            face = t0 if t0 > 0.0 else t1  # towards the hub
            v = face + (r if t0 > 0.0 else -r)
            for k in range(count):
                a = 2.0 * math.pi * k / count
                out.append(sphere(r, ((radius - r) * math.cos(a),
                                      (radius - r) * math.sin(a), v)))
        return out

    def solid_extent(part):
        return [(t0, t1, fit_circle(loop)[1])
                for _, loop, t0, t1, _ in part.solids]

    rail_middle = 0.5 * (RAIL_INNER + RAIL_OUTER)
    # sprocket: a tread under the bushings and a pin between each two seats
    seats = sides[1]['seats']
    angles = np.sort(np.arctan2(seats[:, 1], seats[:, 0]))
    half = math.pi / teeth
    pin_radius = (2.0 * seat_radius * math.sin(0.5 * half) - bushing_radius -
                  PIN_CLEARANCE)
    sprocket_contact = [tread(seat_radius - bushing_radius - 2 * TREAD_GAP)]
    for k in range(teeth):
        a = angles[0] + half + 2.0 * half * k
        sprocket_contact.append(sphere(
            pin_radius, (seat_radius * math.cos(a),
                         seat_radius * math.sin(a), 0.0)))
    # idler: flanges outside the link plates, and a tread under the closest
    # bushing to it. The mesh's runs leave the idler tangent to its flanges,
    # inside the pitch circle of the bushings round it, so the first bushing
    # of each run is closer to it
    idler_reach = min(
        float(np.min(np.linalg.norm(sides[side]['centers'] -
                                    np.array(sides[side]['wheels']['idler'][0]),
                                    axis=1)))
        for side in (1, -1))
    idler_tread = min(idler_pitch, idler_reach) - bushing_radius - TREAD_GAP
    idler_contact = [tread(idler_tread)] + flanges(parts['idler'],
                                                   IDLER_FLANGE_SPHERES)
    roller_solids = solid_extent(parts['track_roller'])
    roller_tread = min(roller_solids, key=lambda e: e[2])
    roller_contact = [sphere(roller_tread[2] - TREAD_GAP,
                             (0.0, 0.0, side * rail_middle))
                      for side in (-1.0, 1.0)] + flanges(
        parts['track_roller'], ROLLER_FLANGE_SPHERES)
    carrier = parts['carrier_roller'].solids[0]
    carrier_contact = [sphere(fit_circle(carrier[1])[1] - TREAD_GAP,
                              (0.0, 0.0, side * rail_middle))
                       for side in (-1.0, 1.0)]
    # link: the bushing at its pin, the rails on the rollers
    link_contact = [sphere(bushing_radius, (PITCH, 0.0, 0.0))]
    for side in (-1.0, 1.0):
        link_contact.append({
            'type': 'box', 'material': 'steel',
            'half_extents': vector([0.5 * PITCH, 0.5 * (RAIL_OUTER -
                                                        RAIL_INNER), -rail]),
            'pose': {'T': vector([0.5 * PITCH, side * rail_middle, 0.0])}})
    # the shoe, from the plate's underside to the grousers' tips, for the
    # sand to press on (soil_contact); no contact with the parts
    link_contact.append({
        'name': 'shoe', 'type': 'box', 'contact': False,
        'half_extents': vector([0.5 * PITCH, 0.5 * shoe_width,
                                0.5 * (tips - plate)]),
        'pose': {'T': vector([shoe_center[0], 0.0, 0.5 * (tips + plate)])}})

    link_com, link_inertia = mass_of(link)
    link_inertia = scaled(link_inertia, LINK_MASS)
    bodies, joints, track_names = [], [], {}
    frames = [f for b in upper['bodies'] if b['name'] == 'undercarriage'
              for f in b['frames']]
    parts_mass = 0.0
    recoil = []
    for side, label in ((1, 'left'), (-1, 'right')):
        data = sides[side]
        names = []
        wheel_list = [('sprocket', data['wheels']['sprocket'][0])]
        wheel_list.append(('idler', data['wheels']['idler'][0]))
        wheel_list += [('roller%d' % (k + 1), c) for k, c in
                       enumerate(data['wheels']['track_roller'])]
        wheel_list += [('carrier%d' % (k + 1), c) for k, c in
                       enumerate(data['wheels']['carrier_roller'])]
        for kind, center in wheel_list:
            name = '%s_%s' % (label, kind)
            part = parts['sprocket' if kind == 'sprocket' else
                         'idler' if kind == 'idler' else
                         'track_roller' if kind.startswith('roller') else
                         'carrier_roller']
            contact = {'excavator_sprocket': sprocket_contact,
                       'excavator_idler': idler_contact,
                       'excavator_track_roller': roller_contact,
                       'excavator_carrier_roller': carrier_contact}[part.name]
            com, mass = mass_of(part)
            mass = scaled(mass, WHEEL_MASSES[part.name])
            parts_mass += mass['mass']
            frames.append(wheel_frame(name + '_axle', center, side))
            bodies.append({'name': name,
                           'geometry': [{'type': 'cad',
                                         'filename': part.name + '.step'}],
                           'collision_geometry': contact,
                           'center_of_mass': com, 'inertia': mass})
            joint = {'name': name + '_joint', 'type': 'revolute',
                     'parent': [['undercarriage'], name + '_axle'],
                     'child': [[name], 'origin'], 'axis': [0.0, 0.0, 1.0]}
            if kind == 'sprocket':
                joint['initial_displacement'] = number(data['sprocket_phase'])
            if kind == 'idler':
                # the idler's yoke slides on the track frame, away from the
                # sprocket, pushed by the recoil spring
                ahead = np.array(center) - np.array(
                    data['wheels']['sprocket'][0])
                ahead /= np.linalg.norm(ahead)
                yoke = '%s_idler_yoke' % label
                axle = frames[-1]
                spring_at = np.array(axle['T']) - IDLER_SPRING_LENGTH * \
                    (wheel_rotation(0.0)[:, :2] @ ahead)
                frames.append({'name': '%s_idler_spring' % label,
                               'T': vector(spring_at), 'R': axle['R']})
                parts_mass += IDLER_YOKE_MASS
                bodies.append({
                    'name': yoke,
                    '//': 'the yoke of the idler and the rod of its recoil '
                          'spring, a block 0.3 x 0.25 x 0.5 m',
                    'center_of_mass': [0.0, 0.0, 0.0],
                    'inertia': inertia(IDLER_YOKE_MASS, np.diag(
                        [0.25 ** 2 + 0.5 ** 2, 0.3 ** 2 + 0.5 ** 2,
                         0.3 ** 2 + 0.25 ** 2]) * IDLER_YOKE_MASS / 12.0)})
                joints.append({'name': yoke + '_joint', 'type': 'prismatic',
                               'parent': [['undercarriage'], name + '_axle'],
                               'child': [[yoke], 'origin'],
                               'axis': vector([ahead[0], ahead[1], 0.0])})
                joint['parent'] = [[yoke], 'origin']
                recoil.append({
                    '//': 'the recoil spring of the track adjuster: it pushes '
                          'the idler forward and so tensions the track',
                    'type': 'spring_damper', 'name': '%s_recoil' % label,
                    'frames': [['undercarriage', '%s_idler_spring' % label],
                               [yoke, 'origin']],
                    'free_length': IDLER_SPRING_LENGTH,
                    'stiffness': IDLER_SPRING_STIFFNESS,
                    'preload': -IDLER_SPRING_PRELOAD,
                    'damping': IDLER_SPRING_DAMPING})
            joints.append(joint)
        # the links, the first on a float joint, each next one turned about
        # the previous one's pin, the last pinned to the first
        link_poses = link_frames(data['centers'], side * xc)
        for i, (origin, rot) in enumerate(link_poses):
            name = '%s_link%d' % (label, i + 1)
            names.append(name)
            parts_mass += link_inertia['mass']
            bodies.append({'name': name,
                           'frames': [{'name': 'pin',
                                       'T': [PITCH, 0.0, 0.0]}],
                           'geometry': [{'type': 'cad', 'filename':
                                         'excavator_track_link.step'}],
                           'collision_geometry': link_contact,
                           'center_of_mass': link_com,
                           'inertia': link_inertia})
        origin, rot = link_poses[0]
        joints.append({'name': '%s_link1_joint' % label, 'type': 'float',
                       'parent': [['fixed'], 'origin'],
                       'child': [['%s_link1' % label], 'origin'],
                       'initial_displacement': {
                           'T': vector(to_model(origin)),
                           'R': rows(rotation @ rot)}})
        for i in range(len(link_poses)):
            a = link_poses[i][1][:, 0]
            b = link_poses[(i + 1) % len(link_poses)][1][:, 0]
            # about the link's y, the mesh's x
            turn = math.atan2(a[1] * b[2] - a[2] * b[1], a[1:] @ b[1:])
            child = names[(i + 1) % len(names)]
            joints.append({
                'name': ('%s_joint' % child) if i + 1 < len(names) else
                        '%s_closing_joint' % label,
                'type': 'revolute', 'parent': [[names[i]], 'pin'],
                'child': [[child], 'origin'], 'axis': [0.0, 1.0, 0.0],
                'initial_displacement': number(turn)})
        track_names[label] = names

    # the undercarriage's frame: what is left of its mass
    frame_mass = UNDERCARRIAGE_MASS - parts_mass
    density = frame_mass / FRAME_HULL_VOLUME
    i = [density * x for x in FRAME_HULL_INERTIA]
    undercarriage = {
        '//': 'the car body and the track frames: the undercarriage of '
              'excavator.json without its tracks, sprockets, idlers and '
              'rollers (written by %s), and the rest of its %.0f kg; '
              'the center of mass and inertia of a uniform solid of the '
              'convex hull of its mesh' % (SOURCE, UNDERCARRIAGE_MASS),
        'name': 'undercarriage',
        'geometry': [{'type': 'mesh', 'texture_mapping': 'triplanar',
                      'texture_scale': 0.5,
                      'filename': ['undercarriage_frame.obj',
                                   'undercarriage.mtl']}],
        'frames': frames,
        'center_of_mass': vector(FRAME_HULL_CENTER),
        'inertia': {'mass': number(frame_mass), 'Ixx': number(i[0]),
                    'Iyy': number(i[1]), 'Izz': number(i[2]),
                    'Ixy': number(i[3]), 'Iyz': number(i[4]),
                    'Izx': number(i[5])}}
    upper_bodies = [b for b in upper['bodies'] if b['name'] != 'undercarriage']
    # the masses of the 323's specifications (above), each body's inertia scaled
    # with its mass
    targets = {'cabin': UPPER_MASS}
    for group, total in FRONT_MASSES:
        old = sum(b['inertia']['mass'] for b in upper_bodies
                  if b['name'] in group)
        for b in upper_bodies:
            if b['name'] in group:
                targets[b['name']] = total * b['inertia']['mass'] / old
    upper_bodies = [dict(b, inertia=scaled(b['inertia'], targets[b['name']]))
                    if b['name'] in targets else b for b in upper_bodies]
    w, l, h = COUNTERWEIGHT_SIZE
    upper_bodies.append({
        'name': 'counterweight',
        '//': 'the counterweight at the back of the upper structure (drawn '
              'by cabin.obj), a block %g x %g x %g m' % (w, l, h),
        'center_of_mass': [0.0, 0.0, 0.0],
        'inertia': inertia(COUNTERWEIGHT_MASS, np.diag(
            [l ** 2 + h ** 2, w ** 2 + h ** 2, w ** 2 + l ** 2]) *
            COUNTERWEIGHT_MASS / 12.0)})
    upper_joints = []
    for j in upper['joints']:
        j = dict(j)
        if j['name'] == 'A_fixed_to_undercarriage':
            j = {'//': 'the undercarriage free, on its tracks: the mesh\'s '
                       'z up turned down, the idlers forward',
                 'name': 'A_fixed_to_undercarriage', 'type': 'float',
                 'parent': [['fixed'], 'origin'],
                 'child': [['undercarriage'], 'origin'],
                 'initial_displacement': {'R': rows(rotation),
                                          'T': vector(translation)}}
        elif j['name'] == 'B_undercarriage_to_cabin':
            j['initial_velocity'] = 0.0
        upper_joints.append(j)
    upper_joints.append({
        'name': 'cabin_to_counterweight', 'type': 'rigid',
        'parent': [['cabin'], 'origin'], 'child': [['counterweight'], 'origin'],
        'initial_displacement': {'R': rows(np.eye(3)),
                                 'T': vector(COUNTERWEIGHT_CENTER)}})

    contact = {
        '//': 'steel on steel, a hundredth of its Young\'s modulus, between '
              'the primitive shapes of the parts: spheres at the links\' '
              'bushings on a flat ellipsoid as the tread of each sprocket '
              'and idler and between pins at the sprocket\'s seats; boxes '
              'as the links\' rails under spheres as the rollers\' treads; '
              'rings of spheres as the flanges of the idlers and the track '
              'rollers either side of the rails, which guide the tracks',
        'name': 'contact', 'type': 'contact',
        'disabled_pairs': [[names[i], names[(i + 1) % len(names)]]
                           for names in track_names.values()
                           for i in range(len(names))],
        'materials': [{'name': 'steel', 'youngs_modulus': 2e9,
                       'poisson_ratio': 0.3, 'restitution': 0.3,
                       'friction': 0.3, 'contact_stiffness': 1e9}]}
    soil = {
        '//': 'the shoes on dry sand: sinkage, traction and side force '
              '(shoe_center the middle of the shoe on the plane of the '
              'grousers\' tips)',
        'name': 'soil', 'type': 'track_soil',
        'soil': {'name': 'dry_sand', 'wall_friction': 0.24},
        'shoes': track_names['left'] + track_names['right'],
        'shoe_center': vector(shoe_center),
        'shoe_normal': [0.0, 0.0, 1.0], 'shoe_along': [1.0, 0.0, 0.0],
        'shoe_length': PITCH, 'shoe_width': number(shoe_width),
        'grouser_height': number(tips - plate)}
    soil_contact = {
        '//': 'the sand carries the shoes and takes their ruts',
        'name': 'soil_contact', 'type': 'soil_contact',
        'bodies': track_names['left'] + track_names['right'],
        'geometry': ['shoe']}
    force_modules = [holding(m) for m in upper['force_modules']] + [
        contact, soil_contact, soil] + recoil

    sensors = [{'type': 'joint', 'name': '%s_sprocket_speed' % label,
                'joint': '%s_sprocket_joint' % label, 'quantity': 'velocity'}
               for label in ('left', 'right')]
    sensors.append({'type': 'joint', 'name': 'swing_angle',
                    'joint': 'B_undercarriage_to_cabin',
                    'quantity': 'position'})
    controllers = []
    for k, label in enumerate(('left', 'right')):
        controllers.append({
            'type': 'pid', 'name': '%s_drive' % label, 'kp': 60000.0,
            'ki': 120000.0, 'kd': 0.0, 'target': 0.0,
            'sensor': '%s_sprocket_speed' % label,
            'joint': '%s_sprocket_joint' % label})
    for k, label in enumerate(('left', 'right')):
        points = []
        for row in DRIVE:
            points += [row[0], row[1 + k]]
        controllers.append({
            'type': 'schedule', 'name': '%s_travel' % label,
            'interpolation': 'cubic', 'points': points,
            'targets': ['controllers/%s_drive/target' % label]})
    controllers.insert(0, {
        '//': 'the swing brake: holds the upper structure on the '
              'undercarriage',
        'type': 'pid', 'name': 'swing_brake', 'kp': 2e6, 'ki': 0.0,
        'kd': 4e5, 'target': 0.0, 'sensor': 'swing_angle',
        'joint': 'B_undercarriage_to_cabin'})
    controllers[1]['//'] = (
        'the travel motors: each sprocket\'s speed held to its target, '
        'which follows a schedule: settle for %g s, forward at %g rad/s '
        '(%.2f m/s), turn on the spot to the left, forward again, stop'
        % (DRIVE[1][0], FORWARD, FORWARD * seat_radius))

    params = dict(original['model_parameters'])
    params['include_directories'] = ['.']
    params['enable_contact_detection'] = True
    params['integrators'] = [{
        '//': 'tighter tolerances take ODE113 tiny steps through the '
              'contacts of the tracks for the same answer',
        'type': 'ode113', 'time_span': TIME_SPAN, 'time_step': 0.02,
        'relative_error': 1e-4, 'absolute_error': 1e-6}]
    params.pop('loggers', None)
    sand_terrain = {
        '//': 'deformable dry sand under the drive (x and y in the ground\'s '
              'frame); the machine settles into it over the first %g s' %
              SAND_SETTLE_TIME,
        'type': 'scm', 'name': 'sand',
        'spacing': SAND_SPACING, 'settle_time': SAND_SETTLE_TIME,
        'soil': {'name': 'dry_sand', 'wall_friction': 0.24},
        'texture_filename': SAND_TEXTURE[0],
        'normal_map_filename': SAND_TEXTURE[1], 'texture_scale': 0.5}
    ground_frame, ground_terrain = ground(out, sand_terrain)
    model = {
        '//': 'The excavator of excavator.json on tracks that drive it, on '
              'dry sand (written by %s). The tracks, sprockets, idlers and '
              'rollers are STEP parts taken from the undercarriage mesh: '
              '%d links a side (excavator_track_link.step), a sprocket of '
              '%d teeth, an idler, %d track rollers and %d carrier rollers, '
              'placed as the mesh has them; their contacts are on primitive '
              'shapes. The hydraulic cylinders hold the arm, the swing '
              'brake the upper structure. A benchmark, not run by meson '
              'test.' % (SOURCE, len(track_names['left']), teeth,
                         len(sides[1]['wheels']['track_roller']),
                         len(sides[1]['wheels']['carrier_roller'])),
        'model_parameters': params,
        # lit by the sun alone: excavator.json's point lights, close over
        # the origin, throw shadows far larger than the machine as it drives
        'frames': [f for f in original.get('frames', [])
                   if f['name'] != 'terrain' and
                   not f['name'].startswith('overhead')] + [ground_frame],
        'terrain': [ground_terrain],
        'assemblies': [{
            'name': 'excavator',
            'bodies': [undercarriage] + upper_bodies + bodies,
            'joints': upper_joints + joints,
            'force_modules': force_modules,
            'sensors': sensors,
            'controllers': controllers}]}
    with open(os.path.join(out, 'excavator_drive.json'), 'w') as f:
        f.write(dump(model) + '\n')
    print('excavator_drive.json: frame %.0f kg, parts %.0f kg; shoe %s, '
          'width %.4f, grousers %.4f; pitch radii %.5f %.5f; pins %.4f' %
          (frame_mass, parts_mass, np.round(shoe_center, 4), shoe_width,
           tips - plate, seat_radius, idler_pitch, pin_radius))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'examples',
        'excavator')
    mesh, xc, sides, link, parts, teeth = extract(out)
    for kind, part in [('link', link)] + list(parts.items()):
        m, com, inertia = part.mass()
        count = (len(part.solids) if isinstance(part, Part) else
                 len(part.faces()))
        print('%s: %d %s, %.2f kg, center of mass %s, inertia %s' %
              (kind, count, 'solids' if isinstance(part, Part) else 'faces',
               m, np.round(com, 4), np.round(np.diag(inertia), 4)))
    print('tracks at x = +-%.4f, %d and %d links; sprocket of %d teeth, '
          'turned %.6f on the - side' %
          (xc, len(sides[1]['centers']), len(sides[-1]['centers']), teeth,
           sides[-1]['sprocket_phase']))

    # the STEP files
    link.write(os.path.join(out, 'excavator_track_link.step'),
               'One link of the track of examples/excavator/excavator_drive'
               '.json, from the undercarriage mesh: the shoe (one grouser) '
               'and the cranked link plates welded to it, a solid of the '
               'mesh\'s planar faces, and the bushing at its pin, a prism. '
               'Origin at the axis of the hinge with the previous link, x '
               'towards the pin (%.0f mm), y along it, z out of the track '
               'loop. In millimeters.' % (1000.0 * PITCH), PLATE_COLOR)
    descriptions = {
        'sprocket': 'The drive sprocket of %d teeth' % teeth,
        'idler': 'The idler, a hub between two flanges',
        'track_roller': 'A track roller, a tread between two flanges',
        'carrier_roller': 'A carrier roller',
    }
    for kind, part in parts.items():
        part.write(os.path.join(out, part.name + '.step'),
                   '%s of examples/excavator/excavator_drive.json, from the '
                   'undercarriage mesh: prisms of its cross sections. Axle '
                   'along z through the origin, x forward and y up as on '
                   'the machine. In millimeters.' % descriptions[kind])

    # the frame: the mesh without the parts
    placed = []
    for side in (1, -1):
        for origin, rotation in link_frames(sides[side]['centers'],
                                            side * xc):
            placed.append((link, origin, rotation, 0.45))
        for kind, centers in sides[side]['wheels'].items():
            phase = sides[side]['sprocket_phase'] if kind == 'sprocket' \
                else 0.0
            for c in centers:
                placed.append((parts[kind], np.array([side * xc, c[0], c[1]]),
                               wheel_rotation(phase), 0.5))
    dropped, total = write_frame(mesh, placed,
                                 os.path.join(out, 'undercarriage_frame.obj'))
    print('undercarriage_frame.obj: %d of %d faces left' %
          (total - dropped, total))
    build_model(out, mesh, xc, sides, link, parts, teeth)
    return mesh, xc, sides, link, parts, teeth


if __name__ == '__main__':
    main()
