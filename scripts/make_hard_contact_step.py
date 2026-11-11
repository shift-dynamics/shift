#!/usr/bin/env python3
"""Write the STEP files of the parts for the tests of contact in exceptional
cases (test/contact/test_cad_hard_contact.cc). In millimeters, z down: a part's
top is its face towards -z.

examples/cad/ball.step: a sphere of 10 mm radius about the origin, two
hemispheres of a spherical surface whose poles are on z, split at its equator.

examples/cad/seat.step: a block 60 mm square and 30 mm deep, its top at z = 0,
with a conical seat of 45 degrees half angle about z, from 14 mm radius at the
top to 6 mm at z = 8 mm, over a bore of 6 mm radius through the block. A ball
of 10 mm radius rests in the seat on a circle.

examples/cad/socket.step: a block 50 mm square and 25 mm deep with a
hemispherical socket of 10.01 mm radius about the origin, at its top: 10 um of
clearance about the ball.

examples/cad/bowl.step: a block 180 mm square and 50 mm deep with a spherical
bowl of 100 mm radius whose center is 70 mm above the top: 71.4 mm in radius at
the top and 30 mm deep.

Each cavity and bore is split into halves at the xz plane; the faces of the
socket and the bowl meet at the pole at the bottom.

examples/cad/eggcrate.step: one face, a bicubic B-spline surface over x from
-20 to 110 mm and y from -20 to 80 mm, of z = -2 g(x) g(y) mm, g(s) =
(1 + cos(2 pi s / 30)) / 2, from control points every 5 mm: twelve bumps, at x
= 0, 30, 60, 90 mm and y = 0, 30, 60 mm, alike since each lies 4 spans or more
from the ends of the knots.

examples/cad/sheet.step: one face, a flat bicubic B-spline surface 150 by 120
mm about the origin, at z = 0, its normal down (+z): a plate's underside.

examples/cad/slab.step: a box 150 by 120 by 5 mm, its underside at z = 0 and
its top at z = -5 mm, of plane faces.

examples/cad/spline_hole.step: a flat bicubic B-spline face 80 mm square about
the origin at z = 0, its normal up, trimmed by a hole of 6 mm radius, and the
two halves of the bore, 20 mm deep. Open below.

examples/cad/shell.step: a dome of B-spline faces, 1 mm thick: its top over x
and y from -75 to 75 mm, z = -8 + 8 (x^2 + y^2) / 75^2 mm, and its underside 1
mm below. Open at its edges.

examples/cad/bolt.step and nut.step: a square thread of 4 mm lead about z,
each turn rising (towards -z) by the lead as it turns from x to y. The bolt's
thread, 2 mm wide, runs from 8 mm to 10 mm radius over 6 turns; the nut's,
1.9 mm wide, from 8.2 mm to 10.2 mm over 3 turns, in the bolt's groove with
0.05 mm of clearance either side. Each flank, crest and root is a face, its
helices cubic B-splines interpolating the helix at every 7.5 degrees. The bolt's
shank goes on 2 mm above its thread and 8 mm below, to flat ends; the nut's
body goes out to 14 mm radius, its ends helical, the top flank of its last turn
and the root of its first carried out to it.

examples/cad/drum.step: a brake drum, a cylinder of 50 mm radius about y from
y = -20 to 20 mm, its side two halves split at x = +-50 mm, its top half facing
-z.

examples/cad/brake_shoe.step: a brake shoe on top of the drum (towards -z):
the part of a ring about y from 50 mm (its inner face, the drum's radius) to 58
mm radius, within 45 degrees of -z, from y = -10 to 10 mm. Its inner face fits
the drum exactly.

examples/cad/eggcrate_cover.step: one face, the surface of eggcrate.step with
its normal down (+z): a cover that fits the bumps exactly.

examples/cad/groove.step: one face, a groove of the section of channel.step
(make_channel_step.py) along a helix of 150 mm radius about z that descends 18
mm per radian over 270 degrees, the path interpolated as the threads' are.

Usage: make_hard_contact_step.py [output directory] (default examples/cad)
"""

import math
import os
import sys

import numpy

from make_channel_step import section
from make_track_step import circle_loop, extrude, polygon_loop, write

BALL = 10.0
GRAY = (0.55, 0.57, 0.60)
GOLD = (0.75, 0.62, 0.30)


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def scale(a, s):
    return tuple(x * s for x in a)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def unit(a):
    return scale(a, 1.0 / math.sqrt(dot(a, a)))


def uniform_knots(count, degree=3):
    spans = count - degree
    return ([0.0] * (degree + 1) + [i / spans for i in range(1, spans)] +
            [1.0] * (degree + 1))


def basis(knots, degree, i, t):
    """B-spline basis function i of degree at t, Cox-de Boor"""
    if degree == 0:
        if knots[i] <= t < knots[i + 1]:
            return 1.0
        if t == knots[-1] and knots[i] < t <= knots[i + 1]:
            return 1.0
        return 0.0
    out = 0.0
    if knots[i + degree] > knots[i]:
        out += ((t - knots[i]) / (knots[i + degree] - knots[i]) *
                basis(knots, degree - 1, i, t))
    if knots[i + degree + 1] > knots[i + 1]:
        out += ((knots[i + degree + 1] - t) /
                (knots[i + degree + 1] - knots[i + 1]) *
                basis(knots, degree - 1, i + 1, t))
    return out


def curve_point(points, knots, degree, t, weights=None):
    w = weights or [1.0] * len(points)
    b = [basis(knots, degree, i, t) * w[i] for i in range(len(points))]
    total = sum(b)
    return tuple(sum(b[i] * p[k] for i, p in enumerate(points)) / total
                 for k in range(3))


def interpolate(values, degree=3):
    """the control points of the clamped B-spline on uniform knots that passes
    through values (a list of tuples) at the Greville abscissae"""
    count = len(values)
    knots = uniform_knots(count, degree)
    params = [sum(knots[i + 1:i + degree + 1]) / degree for i in range(count)]
    matrix = numpy.array([[basis(knots, degree, i, t) for i in range(count)]
                          for t in params])
    solved = numpy.linalg.solve(matrix, numpy.array(values))
    return [tuple(float(x) for x in row) for row in solved], knots, params


class Part:
    """the faces of a part and the edges they share. Each edge keeps sample
    points from its start to its end, and each face's loops are turned to run
    counterclockwise about a given outward direction (the outer loop) or
    clockwise (holes), by the normal of their polygons"""

    def __init__(self, step):
        self.step = step
        self.faces = []

    def vertex(self, p):
        return (self.step.add("VERTEX_POINT('',#%d)" % self.step.point(p)), p)

    def edge(self, start, end, curve, samples):
        e = self.step.add("EDGE_CURVE('',#%d,#%d,#%d,.T.)" %
                          (start[0], end[0], curve))
        return (e, samples)

    def line(self, start, end):
        p, q = start[1], end[1]
        length = math.dist(p, q)
        curve = self.step.add("LINE('',#%d,#%d)" % (
            self.step.point(p), self.step.add("VECTOR('',#%d,%s)" % (
                self.step.direction(sub(q, p)), self.step.num(length)))))
        return self.edge(start, end, curve, [p, q])

    def arc(self, start, end, center, axis):
        """the arc about axis through center from start to end,
        counterclockwise about axis"""
        r = math.dist(start[1], center)
        x = unit(sub(start[1], center))
        y = cross(unit(axis), x)
        e = sub(end[1], center)
        angle = math.atan2(dot(e, y), dot(e, x))
        if angle <= 1e-12:
            angle += 2.0 * math.pi
        samples = [add(center, add(scale(x, r * math.cos(angle * s / 16)),
                                   scale(y, r * math.sin(angle * s / 16))))
                   for s in range(17)]
        curve = self.step.add("CIRCLE('',#%d,%s)" % (
            self.step.placement(center, axis, x), self.step.num(r)))
        return self.edge(start, end, curve, samples)

    def spline(self, start, end, points, knots, degree=3, weights=None):
        distinct = sorted(set(knots))
        ids = ','.join('#%d' % self.step.point(p) for p in points)
        mults = ','.join(str(knots.count(k)) for k in distinct)
        values = ','.join(self.step.num(k) for k in distinct)
        if weights:
            curve = self.step.add(
                "(BOUNDED_CURVE()B_SPLINE_CURVE(%d,(%s),.UNSPECIFIED.,.F.,.F.)"
                "B_SPLINE_CURVE_WITH_KNOTS((%s),(%s),.UNSPECIFIED.)CURVE()"
                "GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE((%s))"
                "REPRESENTATION_ITEM(''))" % (
                    degree, ids, mults, values,
                    ','.join(repr(w) for w in weights)))
        else:
            curve = self.step.add(
                "B_SPLINE_CURVE_WITH_KNOTS('',%d,(%s),.UNSPECIFIED.,.F.,.F.,"
                "(%s),(%s),.UNSPECIFIED.)" % (degree, ids, mults, values))
        samples = [curve_point(points, knots, degree, s / 64.0, weights)
                   for s in range(65)]
        return self.edge(start, end, curve, samples)

    def surface_plane(self, origin, normal, x):
        return self.step.add("PLANE('',#%d)" %
                             self.step.placement(origin, normal, x))

    def surface_cylinder(self, origin, axis, x, radius):
        return self.step.add("CYLINDRICAL_SURFACE('',#%d,%s)" % (
            self.step.placement(origin, axis, x), self.step.num(radius)))

    def surface_cone(self, origin, axis, x, radius, semi_angle):
        return self.step.add("CONICAL_SURFACE('',#%d,%s,%r)" % (
            self.step.placement(origin, axis, x), self.step.num(radius),
            semi_angle))

    def surface_sphere(self, center, radius):
        return self.step.add("SPHERICAL_SURFACE('',#%d,%s)" % (
            self.step.placement(center, (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
            self.step.num(radius)))

    def surface_bspline(self, rows, u_knots, v_knots, u_degree=3, v_degree=3):
        su, sv = sorted(set(u_knots)), sorted(set(v_knots))
        return self.step.add(
            "B_SPLINE_SURFACE_WITH_KNOTS('',%d,%d,(%s),.UNSPECIFIED.,.F.,.F.,"
            ".F.,(%s),(%s),(%s),(%s),.UNSPECIFIED.)" % (
                u_degree, v_degree,
                ','.join('(%s)' % ','.join('#%d' % self.step.point(p)
                                           for p in row) for row in rows),
                ','.join(str(u_knots.count(k)) for k in su),
                ','.join(str(v_knots.count(k)) for k in sv),
                ','.join(self.step.num(k) for k in su),
                ','.join(self.step.num(k) for k in sv)))

    @staticmethod
    def chain(loop):
        """the sample points of a loop of (edge, forward), in its order"""
        out = []
        for (_, samples), forward in loop:
            out.extend(samples if forward else list(reversed(samples)))
        return out

    @staticmethod
    def newell(points):
        n = [0.0, 0.0, 0.0]
        for i, p in enumerate(points):
            q = points[(i + 1) % len(points)]
            n[0] += (p[1] - q[1]) * (p[2] + q[2])
            n[1] += (p[2] - q[2]) * (p[0] + q[0])
            n[2] += (p[0] - q[0]) * (p[1] + q[1])
        return tuple(n)

    def face(self, surface, same_sense, outward, loops):
        """loops: the outer first, each a list of (edge, forward) in order
        around it, either way"""
        bounds = []
        for k, loop in enumerate(loops):
            turn = dot(self.newell(self.chain(loop)), outward)
            assert abs(turn) > 1e-9, 'a loop square to its outward direction'
            if (turn < 0.0) == (k == 0):
                loop = [(e, not forward) for e, forward in reversed(loop)]
            oriented = [self.step.add("ORIENTED_EDGE('',*,*,#%d,%s)" %
                                      (e[0], '.T.' if forward else '.F.'))
                        for e, forward in loop]
            edges = self.step.add("EDGE_LOOP('',(%s))" %
                                  ','.join('#%d' % e for e in oriented))
            bounds.append(self.step.add("%s('',#%d,.T.)" % (
                'FACE_OUTER_BOUND' if k == 0 else 'FACE_BOUND', edges)))
        self.faces.append(self.step.add(
            "ADVANCED_FACE('',(%s),#%d,%s)" % (
                ','.join('#%d' % b for b in bounds), surface,
                '.T.' if same_sense else '.F.')))

    def face_param(self, surface, same_sense, loop):
        """a face of one loop that runs counterclockwise in (u, v), so about
        du x dv: turned to run about the face's normal"""
        if not same_sense:
            loop = [(e, not forward) for e, forward in reversed(loop)]
        oriented = [self.step.add("ORIENTED_EDGE('',*,*,#%d,%s)" %
                                  (e[0], '.T.' if forward else '.F.'))
                    for e, forward in loop]
        edges = self.step.add("EDGE_LOOP('',(%s))" %
                              ','.join('#%d' % e for e in oriented))
        bound = self.step.add("FACE_OUTER_BOUND('',#%d,.T.)" % edges)
        self.faces.append(self.step.add("ADVANCED_FACE('',(#%d),#%d,%s)" % (
            bound, surface, '.T.' if same_sense else '.F.')))

    def solid(self, name):
        shell = self.step.add("CLOSED_SHELL('',(%s))" %
                              ','.join('#%d' % f for f in self.faces))
        return self.step.add("MANIFOLD_SOLID_BREP('%s',#%d)" % (name, shell))


Z = (0.0, 0.0, 1.0)
X = (1.0, 0.0, 0.0)
UP = (0.0, 0.0, -1.0)


def ball(step, name):
    part = Part(step)
    surface = part.surface_sphere((0.0, 0.0, 0.0), BALL)
    a = part.vertex((BALL, 0.0, 0.0))
    b = part.vertex((-BALL, 0.0, 0.0))
    half = [part.arc(a, b, (0.0, 0.0, 0.0), Z),
            part.arc(b, a, (0.0, 0.0, 0.0), Z)]
    loop = [(half[0], True), (half[1], True)]
    part.face(surface, True, UP, [loop])
    part.face(surface, True, Z, [loop])
    return part.solid(name)


def block(part, half_width, depth, rim, cavity):
    """a block from z = 0 to depth, square of half_width about z, with a
    cavity of revolution about z open at its top: its rim the radius rim at z
    = 0, and cavity(part, rim_vertices) the faces below the rim, given the
    vertices of the rim at +x and -x; returns nothing. The bottom is whole
    unless cavity returns the vertices of a through bore at the bottom"""
    w = half_width
    top = [part.vertex((x, y, 0.0)) for x, y in ((w, w), (-w, w), (-w, -w),
                                                 (w, -w))]
    low = [part.vertex((x, y, depth)) for x, y in ((w, w), (-w, w), (-w, -w),
                                                   (w, -w))]
    top_edges = [part.line(top[i], top[(i + 1) % 4]) for i in range(4)]
    low_edges = [part.line(low[i], low[(i + 1) % 4]) for i in range(4)]
    posts = [part.line(top[i], low[i]) for i in range(4)]
    rim_v = [part.vertex((rim, 0.0, 0.0)), part.vertex((-rim, 0.0, 0.0))]
    rim_e = [part.arc(rim_v[0], rim_v[1], (0.0, 0.0, 0.0), Z),
             part.arc(rim_v[1], rim_v[0], (0.0, 0.0, 0.0), Z)]
    part.face(part.surface_plane((0.0, 0.0, 0.0), UP, X), True, UP,
              [[(e, True) for e in top_edges],
               [(rim_e[0], True), (rim_e[1], True)]])
    for i in range(4):
        side = cross(sub(top[(i + 1) % 4][1], top[i][1]), Z)
        part.face(part.surface_plane(top[i][1], unit(side), unit(sub(
            top[(i + 1) % 4][1], top[i][1]))), True, side,
            [[(top_edges[i], True), (posts[(i + 1) % 4], True),
              (low_edges[i], False), (posts[i], False)]])
    bore = cavity(part, rim_v, rim_e)
    loops = [[(e, True) for e in low_edges]]
    if bore:
        loops.append([(bore[0], True), (bore[1], True)])
    part.face(part.surface_plane((0.0, 0.0, depth), Z, X), True, Z, loops)


def revolved_cavity(part, rim_v, rim_e, profile):
    """the faces of a cavity of revolution: profile, from the rim down, of
    (kind, bottom radius, bottom z, extra), kind 'cone' or 'sphere' (extra its
    center's z and radius); the last ends at a pole (radius 0) or is a through
    bore whose bottom arcs are returned"""
    v, e = rim_v, rim_e
    seams = {}
    for kind, r, z, extra in profile:
        top_r = v[0][1][0]
        top_z = v[0][1][2]
        if r > 0.0:
            nv = [part.vertex((r, 0.0, z)), part.vertex((-r, 0.0, z))]
            ne = [part.arc(nv[0], nv[1], (0.0, 0.0, z), Z),
                  part.arc(nv[1], nv[0], (0.0, 0.0, z), Z)]
        else:
            pole = part.vertex((0.0, 0.0, z))
            nv, ne = [pole, pole], None
        if kind == 'cone':
            if abs(top_r - r) < 1e-12:
                surface = part.surface_cylinder((0.0, 0.0, top_z), Z, X, r)
            else:
                # the cone's radius grows towards -z, up
                semi = math.atan2(top_r - r, z - top_z)
                surface = part.surface_cone((0.0, 0.0, top_z), UP, X, top_r,
                                            semi)
            seam = [part.line(v[k], nv[k]) for k in range(2)]
        else:
            center_z, radius = extra
            surface = part.surface_sphere((0.0, 0.0, center_z), radius)
            # the meridians from the rim down, in the xz plane: about -y at +x
            # and about +y at -x turns x towards +z
            seam = [part.arc(v[0], nv[0], (0.0, 0.0, center_z),
                             (0.0, -1.0, 0.0)),
                    part.arc(v[1], nv[1], (0.0, 0.0, center_z),
                             (0.0, 1.0, 0.0))]
        for half, outward in ((0, (0.0, -1.0, 0.0)), (1, (0.0, 1.0, 0.0))):
            loop = [(e[half], True), (seam[1 - half], True)]
            if ne:
                loop.append((ne[half], False))
            loop.append((seam[half], False))
            # the material is outside: the face's normal into the cavity
            towards = add(outward, (0.0, 0.0, -0.3 if kind == 'sphere' else
                                    0.0))
            part.face(surface, False, towards, [loop])
        v, e = nv, ne
    return e


def seat(step, name):
    part = Part(step)
    block(part, 30.0, 30.0, 14.0, lambda p, v, e: revolved_cavity(
        p, v, e, [('cone', 6.0, 8.0, None), ('cone', 6.0, 30.0, None)]))
    return part.solid(name)


SOCKET = 10.01


def socket(step, name):
    part = Part(step)
    block(part, 25.0, 25.0, SOCKET, lambda p, v, e: revolved_cavity(
        p, v, e, [('sphere', 0.0, SOCKET, (0.0, SOCKET))]))
    return part.solid(name)


BOWL = 100.0
BOWL_CENTER = -70.0


def bowl(step, name):
    part = Part(step)
    rim = math.sqrt(BOWL ** 2 - BOWL_CENTER ** 2)
    block(part, 90.0, 50.0, rim, lambda p, v, e: revolved_cavity(
        p, v, e, [('sphere', 0.0, BOWL_CENTER + BOWL, (BOWL_CENTER, BOWL))]))
    return part.solid(name)


def grid_face(part, xs, ys, height, up):
    """one bicubic B-spline face with control points (x, y, height(x, y)) over
    the grids xs and ys, its normal up (-z) or down, bounded by its four
    boundary curves; returns the face's loops and surface"""
    # u along y and v along x: du x dv is -z
    rows = [[(x, y, height(x, y)) for x in xs] for y in ys]
    if not up:
        rows = list(reversed(rows))
    u_knots, v_knots = uniform_knots(len(rows)), uniform_knots(len(xs))
    surface = part.surface_bspline(rows, u_knots, v_knots)
    corners = [part.vertex(rows[0][0]), part.vertex(rows[0][-1]),
               part.vertex(rows[-1][-1]), part.vertex(rows[-1][0])]
    sides = [part.spline(corners[0], corners[1], rows[0], v_knots),
             part.spline(corners[1], corners[2], [r[-1] for r in rows],
                         u_knots),
             part.spline(corners[3], corners[2], rows[-1], v_knots),
             part.spline(corners[0], corners[3], [r[0] for r in rows],
                         u_knots)]
    loop = [(sides[0], True), (sides[1], True), (sides[2], False),
            (sides[3], False)]
    return surface, loop


def eggcrate(step, name):
    part = Part(step)

    def g(s):
        return 0.5 * (1.0 + math.cos(2.0 * math.pi * s / 30.0))

    surface, loop = grid_face(
        part, [-20.0 + 5.0 * i for i in range(27)],
        [-20.0 + 5.0 * j for j in range(21)],
        lambda x, y: -2.0 * g(x) * g(y), True)
    part.face(surface, True, UP, [loop])
    return part.solid(name)


def sheet(step, name):
    part = Part(step)
    surface, loop = grid_face(
        part, [-75.0 + 10.0 * i for i in range(16)],
        [-60.0 + 10.0 * j for j in range(13)], lambda x, y: 0.0, False)
    part.face(surface, True, Z, [loop])
    return part.solid(name)


def slab(step, name):
    return extrude(step, name, [polygon_loop([(-75.0, -60.0), (75.0, -60.0),
                                              (75.0, 60.0), (-75.0, 60.0)])],
                   (0.0, 0.0, 0.0), X, (0.0, -1.0, 0.0), 5.0)


HOLE = 6.0


def spline_hole(step, name):
    part = Part(step)
    surface, outer = grid_face(
        part, [-40.0 + 10.0 * i for i in range(9)],
        [-40.0 + 10.0 * j for j in range(9)], lambda x, y: 0.0, True)
    rim_v = [part.vertex((HOLE, 0.0, 0.0)), part.vertex((-HOLE, 0.0, 0.0))]
    rim_e = [part.arc(rim_v[0], rim_v[1], (0.0, 0.0, 0.0), Z),
             part.arc(rim_v[1], rim_v[0], (0.0, 0.0, 0.0), Z)]
    part.face(surface, True, UP, [outer, [(rim_e[0], True),
                                          (rim_e[1], True)]])
    low_v = [part.vertex((HOLE, 0.0, 20.0)), part.vertex((-HOLE, 0.0, 20.0))]
    low_e = [part.arc(low_v[0], low_v[1], (0.0, 0.0, 20.0), Z),
             part.arc(low_v[1], low_v[0], (0.0, 0.0, 20.0), Z)]
    seam = [part.line(rim_v[k], low_v[k]) for k in range(2)]
    bore = part.surface_cylinder((0.0, 0.0, 0.0), Z, X, HOLE)
    for half, outward in ((0, (0.0, -1.0, 0.0)), (1, (0.0, 1.0, 0.0))):
        part.face(bore, False, outward,
                  [[(rim_e[half], True), (seam[1 - half], True),
                    (low_e[half], False), (seam[half], False)]])
    return part.solid(name)


def dome(x, y):
    return -8.0 + 8.0 * (x * x + y * y) / 75.0 ** 2


def shell(step, name):
    part = Part(step)
    xs = [-75.0 + 7.5 * i for i in range(21)]
    surface, loop = grid_face(part, xs, xs, dome, True)
    part.face(surface, True, UP, [loop])
    surface, loop = grid_face(part, xs, xs, lambda x, y: dome(x, y) + 1.0,
                              False)
    part.face(surface, True, Z, [loop])
    return part.solid(name)


LEAD = 4.0
STEP_ANGLE = math.radians(7.5)


DRUM = 50.0
DRUM_HALF_WIDTH = 20.0
SHOE_OUTER = 58.0
SHOE_HALF_WIDTH = 10.0
SHOE_HALF_ANGLE = math.radians(45.0)
Y = (0.0, 1.0, 0.0)


def drum(step, name):
    part = Part(step)
    surface = part.surface_cylinder((0.0, 0.0, 0.0), Y, X, DRUM)
    ends = []
    for y in (-DRUM_HALF_WIDTH, DRUM_HALF_WIDTH):
        v = [part.vertex((DRUM, y, 0.0)), part.vertex((-DRUM, y, 0.0))]
        # about y from +x the arc turns towards -z: the top half first
        arcs = [part.arc(v[0], v[1], (0.0, y, 0.0), Y),
                part.arc(v[1], v[0], (0.0, y, 0.0), Y)]
        assert arcs[0][1][8][2] < 0.0, 'the first arc is the top half'
        ends.append((v, arcs))
    seams = [part.line(ends[0][0][k], ends[1][0][k]) for k in range(2)]
    for half, outward in ((0, UP), (1, Z)):
        loop = [(ends[0][1][half], True), (seams[1 - half], True),
                (ends[1][1][half], False), (seams[half], False)]
        part.face(surface, True, outward, [loop])
    for (v, arcs), normal in ((ends[0], (0.0, -1.0, 0.0)), (ends[1], Y)):
        part.face(part.surface_plane(v[0][1], normal, X), True, normal,
                  [[(arcs[0], True), (arcs[1], True)]])
    return part.solid(name)


def brake_shoe(step, name):
    part = Part(step)
    a = SHOE_HALF_ANGLE

    def at(r, angle, y):
        return (r * math.sin(angle), y, -r * math.cos(angle))

    # v[radius][side of the angle][side of y]
    v = [[[part.vertex(at(r, angle, y))
           for y in (-SHOE_HALF_WIDTH, SHOE_HALF_WIDTH)]
          for angle in (-a, a)] for r in (DRUM, SHOE_OUTER)]
    arcs = [[part.arc(v[i][0][k], v[i][1][k],
                      (0.0, (-SHOE_HALF_WIDTH, SHOE_HALF_WIDTH)[k], 0.0),
                      (0.0, -1.0, 0.0)) for k in range(2)] for i in range(2)]
    assert arcs[0][0][1][8][2] < 0.0, 'the arcs pass over the top'
    radial = [[part.line(v[0][j][k], v[1][j][k]) for k in range(2)]
              for j in range(2)]
    along = [[part.line(v[i][j][0], v[i][j][1]) for j in range(2)]
             for i in range(2)]
    for i, (radius, sense, outward) in enumerate(((DRUM, False, Z),
                                                  (SHOE_OUTER, True, UP))):
        surface = part.surface_cylinder((0.0, 0.0, 0.0), Y, X, radius)
        part.face(surface, sense, outward,
                  [[(arcs[i][0], True), (along[i][1], True),
                    (arcs[i][1], False), (along[i][0], False)]])
    for j, angle in enumerate((-a, a)):
        normal = (math.cos(angle) * (1 if j else -1), 0.0,
                  math.sin(angle) * (1 if j else -1))
        part.face(part.surface_plane(v[0][j][0][1], normal, Y), True, normal,
                  [[(radial[j][0], True), (along[1][j], True),
                    (radial[j][1], False), (along[0][j], False)]])
    for k, normal in enumerate(((0.0, -1.0, 0.0), Y)):
        part.face(part.surface_plane(v[0][0][k][1], normal, X), True, normal,
                  [[(arcs[0][k], True), (radial[1][k], True),
                    (arcs[1][k], False), (radial[0][k], False)]])
    return part.solid(name)


def eggcrate_cover(step, name):
    part = Part(step)

    def g(s):
        return 0.5 * (1.0 + math.cos(2.0 * math.pi * s / 30.0))

    surface, loop = grid_face(
        part, [-20.0 + 5.0 * i for i in range(27)],
        [-20.0 + 5.0 * j for j in range(21)],
        lambda x, y: -2.0 * g(x) * g(y), False)
    part.face(surface, True, Z, [loop])
    return part.solid(name)


def helix(turn_from, turn_to):
    """the control points (x, y, z) and knots of the unit helix (radius 1),
    rising (z = -LEAD theta / 2 pi) from turn_from to turn_to turns,
    interpolated every STEP_ANGLE"""
    count = int(round((turn_to - turn_from) * 2.0 * math.pi / STEP_ANGLE)) + 1
    thetas = [2.0 * math.pi * turn_from + i * STEP_ANGLE for i in range(count)]
    # interpolate at the Greville abscissae, mapped to theta linearly
    knots = uniform_knots(count)
    params = [sum(knots[i + 1:i + 4]) / 3.0 for i in range(count)]
    span = thetas[-1] - thetas[0]
    values = [(math.cos(t), math.sin(t), -LEAD * t / (2.0 * math.pi))
              for t in (thetas[0] + p * span for p in params)]
    points, knots, _ = interpolate(values)
    return points, knots


def helical_face(part, points, knots, a, b, outward):
    """the face between two helices on the unit helix's control points:
    a and b are (radius, axial offset along z), a ruled surface between them;
    outward: the solid's outward direction, roughly"""
    def at(c, r, dz):
        return (r * c[0], r * c[1], c[2] + dz)
    rows = [[at(c, *a), at(c, *b)] for c in points]
    surface = part.surface_bspline(rows, knots, [0.0, 0.0, 1.0, 1.0], 3, 1)
    first, last = rows[0], rows[-1]
    corners = [part.vertex(first[0]), part.vertex(last[0]),
               part.vertex(last[1]), part.vertex(first[1])]
    sides = [part.spline(corners[0], corners[1], [r[0] for r in rows], knots),
             part.line(corners[1], corners[2]),
             part.spline(corners[3], corners[2], [r[1] for r in rows], knots),
             part.line(corners[0], corners[3])]
    loop = [(sides[0], True), (sides[1], True), (sides[2], False),
            (sides[3], False)]
    # the outward direction at the middle of the face, and the sense of the
    # surface, du x dv, against it
    middle = len(rows) // 2
    radial = unit((rows[middle][0][0], rows[middle][0][1], 0.0))
    direction = {'up': UP, 'down': Z, 'out': radial,
                 'in': scale(radial, -1.0)}[outward]
    du = sub(rows[middle + 1][0], rows[middle - 1][0])
    dv = sub(rows[middle][1], rows[middle][0])
    part.face_param(surface, dot(cross(du, dv), direction) > 0.0, loop)


def core_face(part, points, knots, radius, dz, z):
    """the face of the cylinder of radius (outward) between the helix at
    radius and dz over one turn and the plane z: bounded by the helix, the
    full circle on the plane, and the two lines of the cylinder at angle 0
    from the helix's ends to the circle"""
    helix_points = [(radius * c[0], radius * c[1], c[2] + dz) for c in points]
    start = part.vertex(helix_points[0])
    end = part.vertex(helix_points[-1])
    point = part.vertex((radius, 0.0, z))
    helix_edge = part.spline(start, end, helix_points, knots)
    circle = part.arc(point, point, (0.0, 0.0, z), Z)
    to_start = part.line(start, point)
    to_end = part.line(end, point)
    # counterclockwise in (u, v) = (angle about z, z), so about du x dv,
    # which is outward
    if z < helix_points[0][2]:
        loop = [(circle, True), (to_end, False), (helix_edge, False),
                (to_start, True)]
    else:
        loop = [(helix_edge, True), (to_end, True), (circle, False),
                (to_start, False)]
    surface = part.surface_cylinder((0.0, 0.0, 0.0), Z, X, radius)
    part.face_param(surface, True, loop)


def disc(part, radius, z, normal):
    """a flat end of radius at z, its outward normal along z either way"""
    a = part.vertex((radius, 0.0, z))
    b = part.vertex((-radius, 0.0, z))
    halves = [part.arc(a, b, (0.0, 0.0, z), Z), part.arc(b, a, (0.0, 0.0, z), Z)]
    part.face(part.surface_plane((0.0, 0.0, z), normal, X), True, normal,
              [[(halves[0], True), (halves[1], True)]])


def bolt(step, name):
    part = Part(step)
    points, knots = helix(0.0, 6.0)
    # the tooth from z to z + 2 below its top flank (z down), at each angle
    helical_face(part, points, knots, (8.0, 0.0), (10.0, 0.0), 'up')
    helical_face(part, points, knots, (10.0, 0.0), (10.0, 2.0), 'out')
    helical_face(part, points, knots, (8.0, 2.0), (10.0, 2.0), 'down')
    helical_face(part, points, knots, (8.0, 2.0), (8.0, LEAD), 'out')
    # the shank beyond the thread, its core cylinder from the last turn of
    # the thread to a flat end 2 mm above it, and from the first turn to a
    # flat end 8 mm below it
    top, top_knots = helix(5.0, 6.0)
    core_face(part, top, top_knots, 8.0, 0.0, -6.0 * LEAD - 2.0)
    disc(part, 8.0, -6.0 * LEAD - 2.0, UP)
    bottom, bottom_knots = helix(0.0, 1.0)
    core_face(part, bottom, bottom_knots, 8.0, LEAD, LEAD + 8.0)
    disc(part, 8.0, LEAD + 8.0, Z)
    return part.solid(name)


def nut(step, name):
    part = Part(step)
    points, knots = helix(1.5, 4.5)
    c = 0.05
    # the nut's tooth in the bolt's groove, from z + 2 + c to z + 4 - c
    helical_face(part, points, knots, (8.2, 2.0 + c), (10.2, 2.0 + c), 'up')
    helical_face(part, points, knots, (8.2, 2.0 + c), (8.2, LEAD - c), 'in')
    helical_face(part, points, knots, (8.2, LEAD - c), (10.2, LEAD - c),
                 'down')
    helical_face(part, points, knots, (10.2, LEAD - c), (10.2, LEAD + 2.0 + c),
                 'in')
    # the body about the thread, out to 14 mm: its ends the top flank of the
    # last turn and the root of the first carried out to it, and its outside
    # between them, 3 leads less the tooth high
    last, last_knots = helix(3.5, 4.5)
    helical_face(part, last, last_knots, (10.2, 2.0 + c), (14.0, 2.0 + c), 'up')
    first, first_knots = helix(1.5, 2.5)
    helical_face(part, first, first_knots, (10.2, LEAD + 2.0 + c),
                 (14.0, LEAD + 2.0 + c), 'down')
    helical_face(part, first, first_knots, (14.0, 2.0 + c - 2.0 * LEAD),
                 (14.0, LEAD + 2.0 + c), 'out')
    return part.solid(name)


GROOVE_RADIUS = 150.0
GROOVE_DROP = 18.0  # mm per radian, down (+z)
GROOVE_TURN = 1.5 * math.pi


def groove(step, name):
    part = Part(step)
    count = 37
    knots = uniform_knots(count)
    params = [sum(knots[i + 1:i + 4]) / 3.0 for i in range(count)]
    values = [(GROOVE_RADIUS * math.cos(t), GROOVE_RADIUS * math.sin(t),
               GROOVE_DROP * t) for t in (p * GROOVE_TURN for p in params)]
    path, knots, _ = interpolate(values)
    profile = section()
    rows = []
    for i, c in enumerate(path):
        t = params[i] * GROOVE_TURN
        tangent = unit((-GROOVE_RADIUS * math.sin(t),
                        GROOVE_RADIUS * math.cos(t), GROOVE_DROP))
        side = unit(cross(UP, tangent))
        top = cross(tangent, side)
        rows.append([add(c, add(scale(side, s), scale(top, h)))
                     for s, h in profile])
    v_knots = uniform_knots(len(profile))
    surface = part.surface_bspline(rows, knots, v_knots)
    corners = [part.vertex(rows[0][0]), part.vertex(rows[-1][0]),
               part.vertex(rows[-1][-1]), part.vertex(rows[0][-1])]
    sides = [part.spline(corners[0], corners[1], [r[0] for r in rows], knots),
             part.spline(corners[1], corners[2], rows[-1], v_knots),
             part.spline(corners[3], corners[2], [r[-1] for r in rows],
                         knots),
             part.spline(corners[0], corners[3], rows[0], v_knots)]
    loop = [(sides[0], True), (sides[1], True), (sides[2], False),
            (sides[3], False)]
    middle = len(rows) // 2
    du = sub(rows[middle + 1][4], rows[middle - 1][4])
    dv = sub(rows[middle][5], rows[middle][3])
    # the groove's face looks up out of the groove at its bottom
    part.face_param(surface, dot(cross(du, dv), UP) > 0.0, loop)
    return part.solid(name)


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'examples', 'cad')
    by = ' In millimeters. Written by scripts/make_hard_contact_step.py'
    parts = [
        ('ball', 'a sphere of %g mm radius, two hemispheres' % BALL, GOLD),
        ('seat', 'a block with a conical seat of 45 degrees half angle over '
         'a bore', GRAY),
        ('socket', 'a block with a hemispherical socket of %g mm radius' %
         SOCKET, GRAY),
        ('bowl', 'a block with a spherical bowl of %g mm radius' % BOWL,
         GRAY),
        ('eggcrate', 'a B-spline face of twelve bumps 2 mm high', GRAY),
        ('sheet', 'a flat B-spline face 150 by 120 mm, its normal down',
         GOLD),
        ('slab', 'a box 150 by 120 by 5 mm', GOLD),
        ('spline_hole', 'a flat B-spline face with a hole of %g mm radius '
         'and its bore' % HOLE, GRAY),
        ('shell', 'a dome of B-spline faces 1 mm thick', GRAY),
        ('bolt', 'a square thread of %g mm lead over 6 turns on a shank' % LEAD,
         GRAY),
        ('nut', 'a nut of 14 mm radius with a square thread over 3 turns',
         GOLD),
        ('groove', 'a groove along a helix of %g mm radius' % GROOVE_RADIUS,
         GRAY),
        ('drum', 'a brake drum of %g mm radius' % DRUM, GRAY),
        ('brake_shoe', 'a brake shoe whose inner face fits the drum of %g mm '
         'radius' % DRUM, GOLD),
        ('eggcrate_cover', 'the B-spline face of eggcrate.step, its normal '
         'down', GOLD),
    ]
    builders = {'ball': ball, 'seat': seat, 'socket': socket, 'bowl': bowl,
                'eggcrate': eggcrate, 'sheet': sheet, 'slab': slab,
                'spline_hole': spline_hole, 'shell': shell, 'bolt': bolt,
                'nut': nut, 'groove': groove, 'drum': drum,
                'brake_shoe': brake_shoe, 'eggcrate_cover': eggcrate_cover}
    only = sys.argv[2:]
    for name, comment, rgb in parts:
        if only and name not in only:
            continue
        write(os.path.join(directory, name + '.step'), name, comment + '.' + by,
              [(name, builders[name], rgb)])


if __name__ == '__main__':
    main()
