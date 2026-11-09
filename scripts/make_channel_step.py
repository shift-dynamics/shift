#!/usr/bin/env python3
"""Write the STEP files of a torus and of a channel swept along a spline, for
the tests of contact between curved faces touching in several places
(test/contact/test_cad_round_contact.cc):

examples/cad/torus.step: a torus of 40 mm major radius and 10 mm minor radius
about z, centered at the origin. It has four faces: each half of the torus,
u (the angle about z) from -90 to 90 degrees and from 90 to 270 degrees, is
split along the tube at v = +-135 degrees (v is the angle about the tube from
the plane of the torus, 0 outward) into an outer face of 270 degrees and an
inner face of 90 degrees. The four points of the tube nearest the flanks and
walls of the channel all lie inside one outer face.

examples/cad/channel.step: a block with a slot along a planar cubic B-spline
path in the xy plane, an S-bend between two straight ends. The slot's section
is one cubic B-spline curve: two walls 23 mm apart and 40 mm high, joined by
flanks at about 45 degrees that meet at its bottom, on z = 0. The slot opens
to -z (up, in a model with z down), and the block's base is at z = 10 mm. The
slot is one face, a B-spline surface whose control points are those of the
section placed at each control point of the path, across the path's normal
there. A torus standing in the slot rests on both flanks, and touches a wall
when it turns or tips.

In millimeters.

Usage: make_channel_step.py [output directory] (default examples/cad)
"""

import math
import os
import sys

from make_contact_step import spline_curve
from make_surface_step import circle
from make_track_step import write

MAJOR = 40.0
MINOR = 10.0
SPLIT = math.radians(135.0)

# the slot: half its width between the walls, the height of the walls above
# its bottom, the half width of the block, and the depth of its base below
# the bottom
HALF_WIDTH = 11.5
HEIGHT = 40.0
BLOCK_HALF_WIDTH = 30.0
BASE = 10.0

# the path, (x, y): straight from x = 0 to 200 mm, an S-bend of 30 mm offset
# along a smoothstep from x = 200 to 600 mm, and straight again to 800 mm
OFFSET = 30.0
PATH = [(x, OFFSET * min(max((x - 200.0) / 400.0, 0.0), 1.0) ** 2 *
         (3.0 - 2.0 * min(max((x - 200.0) / 400.0, 0.0), 1.0)))
        for x in (0.0, 50.0, 100.0, 150.0, 200.0, 250.0, 300.0, 350.0, 400.0,
                  450.0, 500.0, 550.0, 600.0, 650.0, 700.0, 750.0, 800.0)]


def uniform_knots(count, degree=3):
    spans = count - degree
    return ([0.0] * (degree + 1) + [i / spans for i in range(1, spans)] +
            [1.0] * (degree + 1))


def section():
    """the slot's section, (lateral, height) from the left wall's top (+y
    side, lateral > 0) to the right wall's top: lateral along the path's
    left normal"""
    w, h = HALF_WIDTH, HEIGHT
    return [(w, h), (w, 0.5 * (h + w)), (w, w), (0.5 * w, 0.5 * w),
            (0.0, 0.0), (-0.5 * w, 0.5 * w), (-w, w), (-w, 0.5 * (h + w)),
            (-w, h)]


def normals():
    """the unit left normal of the path at each of its control points, from
    the control polygon"""
    out = []
    for i in range(len(PATH)):
        p = PATH[max(i - 1, 0)]
        q = PATH[min(i + 1, len(PATH) - 1)]
        tx, ty = q[0] - p[0], q[1] - p[1]
        n = math.hypot(tx, ty)
        out.append((-ty / n, tx / n))
    return out


def placed(i, lateral, height):
    """the point at lateral and height (up, -z) from the path's control
    point i"""
    n = normals()[i]
    return (PATH[i][0] + lateral * n[0], PATH[i][1] + lateral * n[1],
            -height)


def basis(knots, degree, i, t):
    """the B-spline basis function i of degree at t, Cox-de Boor"""
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


def curve_point(points, knots, t):
    return tuple(sum(basis(knots, 3, i, t) * p[k]
                     for i, p in enumerate(points)) for k in range(3))


def path_radius():
    """the least radius of curvature of the path, sampled"""
    points = [(x, y, 0.0) for x, y in PATH]
    knots = uniform_knots(len(points))
    least = math.inf
    h = 1e-4
    for s in range(1, 1000):
        t = s / 1000.0
        a, b, c = (curve_point(points, knots, t + d) for d in (-h, 0.0, h))
        d1 = [(c[k] - a[k]) / (2 * h) for k in range(2)]
        d2 = [(c[k] - 2 * b[k] + a[k]) / (h * h) for k in range(2)]
        cross = abs(d1[0] * d2[1] - d1[1] * d2[0])
        if cross > 0.0:
            least = min(least, math.hypot(*d1) ** 3 / cross)
    return least


class Shell:
    """the faces of a closed shell, from shared vertices and edges"""

    def __init__(self, step):
        self.step = step
        self.faces = []

    def vertex(self, p):
        return self.step.add("VERTEX_POINT('',#%d)" % self.step.point(p))

    def edge(self, start, end, curve):
        return self.step.add("EDGE_CURVE('',#%d,#%d,#%d,.T.)" %
                             (start, end, curve))

    def line(self, start, end, p, q):
        length = math.dist(p, q)
        direction = tuple((q[k] - p[k]) / length for k in range(3))
        curve = self.step.add("LINE('',#%d,#%d)" % (
            self.step.point(p), self.step.add("VECTOR('',#%d,%s)" % (
                self.step.direction(direction), self.step.num(length)))))
        return self.edge(start, end, curve)

    def face(self, surface, loop, same_sense=True):
        """loop: (edge, forward), counterclockwise about the face's outward
        normal"""
        oriented = [self.step.add("ORIENTED_EDGE('',*,*,#%d,%s)" %
                                  (e, '.T.' if forward else '.F.'))
                    for e, forward in loop]
        bound = self.step.add("FACE_OUTER_BOUND('',#%d,.T.)" %
                              self.step.add("EDGE_LOOP('',(%s))" % ','.join(
                                  '#%d' % e for e in oriented)))
        self.faces.append(self.step.add("ADVANCED_FACE('',(#%d),#%d,%s)" % (
            bound, surface, '.T.' if same_sense else '.F.')))

    def plane(self, origin, normal, x, loop):
        self.face(self.step.add("PLANE('',#%d)" %
                                self.step.placement(origin, normal, x)), loop)

    def solid(self, name):
        shell = self.step.add("CLOSED_SHELL('',(%s))" %
                              ','.join('#%d' % f for f in self.faces))
        return self.step.add("MANIFOLD_SOLID_BREP('%s',#%d)" % (name, shell))


def torus(step, name):
    """four faces, each counterclockwise in (u, v) so that its normal is the
    torus's outward one"""
    shell = Shell(step)
    surface = step.add("TOROIDAL_SURFACE('',#%d,%s,%s)" % (
        step.placement((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
        step.num(MAJOR), step.num(MINOR)))

    def at(u, v):
        r = MAJOR + MINOR * math.cos(v)
        return (r * math.cos(u), r * math.sin(u), MINOR * math.sin(v))

    half = 0.5 * math.pi
    vertex = {(u, v): shell.vertex(at(u, v))
              for u in (half, -half) for v in (SPLIT, -SPLIT)}

    def ring(v):
        """the circle about z of the points at v, its parameter u"""
        return circle(step, (0.0, 0.0, MINOR * math.sin(v)), (0.0, 0.0, 1.0),
                      (1.0, 0.0, 0.0), MAJOR + MINOR * math.cos(v))

    def tube(u):
        """the circle of the tube at u, its parameter v"""
        radial = (math.cos(u), math.sin(u), 0.0)
        return circle(step, (MAJOR * radial[0], MAJOR * radial[1], 0.0),
                      (math.sin(u), -math.cos(u), 0.0), radial, MINOR)

    high, low = ring(SPLIT), ring(-SPLIT)
    # the arcs of the rings about z: A from u = -90 to 90, B from 90 to 270
    high_a = shell.edge(vertex[(-half, SPLIT)], vertex[(half, SPLIT)], high)
    high_b = shell.edge(vertex[(half, SPLIT)], vertex[(-half, SPLIT)], high)
    low_a = shell.edge(vertex[(-half, -SPLIT)], vertex[(half, -SPLIT)], low)
    low_b = shell.edge(vertex[(half, -SPLIT)], vertex[(-half, -SPLIT)], low)
    # the arcs of the tube at u = +-90: outer from v = -135 to 135, inner
    # from 135 to 225
    tube_pos, tube_neg = tube(half), tube(-half)
    out_pos = shell.edge(vertex[(half, -SPLIT)], vertex[(half, SPLIT)],
                         tube_pos)
    in_pos = shell.edge(vertex[(half, SPLIT)], vertex[(half, -SPLIT)],
                        tube_pos)
    out_neg = shell.edge(vertex[(-half, -SPLIT)], vertex[(-half, SPLIT)],
                         tube_neg)
    in_neg = shell.edge(vertex[(-half, SPLIT)], vertex[(-half, -SPLIT)],
                        tube_neg)
    shell.face(surface, [(low_a, True), (out_pos, True), (high_a, False),
                         (out_neg, False)])
    shell.face(surface, [(low_b, True), (out_neg, True), (high_b, False),
                         (out_pos, False)])
    shell.face(surface, [(high_a, True), (in_pos, True), (low_a, False),
                         (in_neg, False)])
    shell.face(surface, [(high_b, True), (in_neg, True), (low_b, False),
                         (in_pos, False)])
    return shell.solid(name)


def bspline_surface(step, rows, u_knots, v_knots, v_degree):
    distinct = [sorted(set(k)) for k in (u_knots, v_knots)]
    return step.add(
        "B_SPLINE_SURFACE_WITH_KNOTS('',3,%d,(%s),.UNSPECIFIED.,.F.,.F.,.F.,"
        "(%s),(%s),(%s),(%s),.UNSPECIFIED.)" % (
            v_degree,
            ','.join('(%s)' % ','.join('#%d' % step.point(p) for p in row)
                     for row in rows),
            ','.join(str(u_knots.count(k)) for k in distinct[0]),
            ','.join(str(v_knots.count(k)) for k in distinct[1]),
            ','.join(step.num(k) for k in distinct[0]),
            ','.join(step.num(k) for k in distinct[1])))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def channel(step, name):
    shell = Shell(step)
    last = len(PATH) - 1
    profile = section()
    u_knots = uniform_knots(len(PATH))
    v_knots = uniform_knots(len(profile))
    w, top = BLOCK_HALF_WIDTH, HEIGHT
    rows = [[placed(i, s, h) for s, h in profile] for i in range(len(PATH))]

    # the slot's outward normal, du x dv, is up (-z) at the middle
    du = [rows[8][4][k] - rows[6][4][k] for k in range(3)]
    dv = [rows[7][5][k] - rows[7][3][k] for k in range(3)]
    assert cross(du, dv)[2] < 0.0

    # vertices at the start (0) and end (1): the tops of the slot's walls,
    # left (+y) and right, the block's top and bottom corners
    v = {}
    for end, i in (('0', 0), ('1', last)):
        v['slot_left' + end] = shell.vertex(placed(i, profile[0][0], top))
        v['slot_right' + end] = shell.vertex(placed(i, profile[-1][0], top))
        v['top_left' + end] = shell.vertex(placed(i, w, top))
        v['top_right' + end] = shell.vertex(placed(i, -w, top))
        v['base_left' + end] = shell.vertex(placed(i, w, -BASE))
        v['base_right' + end] = shell.vertex(placed(i, -w, -BASE))

    def along(lateral, height):
        return spline_curve(step, [placed(i, lateral, height)
                                   for i in range(len(PATH))], u_knots)

    def across(i):
        return spline_curve(step, rows[i], v_knots)

    rim_left = shell.edge(v['slot_left0'], v['slot_left1'],
                          along(profile[0][0], top))
    rim_right = shell.edge(v['slot_right0'], v['slot_right1'],
                           along(profile[-1][0], top))
    section0 = shell.edge(v['slot_left0'], v['slot_right0'], across(0))
    section1 = shell.edge(v['slot_left1'], v['slot_right1'], across(last))
    top_left = shell.edge(v['top_left0'], v['top_left1'], along(w, top))
    top_right = shell.edge(v['top_right0'], v['top_right1'], along(-w, top))
    base_left = shell.edge(v['base_left0'], v['base_left1'], along(w, -BASE))
    base_right = shell.edge(v['base_right0'], v['base_right1'],
                            along(-w, -BASE))
    lines = {}
    for end, i in (('0', 0), ('1', last)):
        for a, b, pa, pb in (
                ('slot_left', 'top_left', (profile[0][0], top), (w, top)),
                ('slot_right', 'top_right', (profile[-1][0], top),
                 (-w, top)),
                ('top_left', 'base_left', (w, top), (w, -BASE)),
                ('top_right', 'base_right', (-w, top), (-w, -BASE)),
                ('base_left', 'base_right', (w, -BASE), (-w, -BASE))):
            lines[a + '-' + b + end] = shell.line(
                v[a + end], v[b + end], placed(i, *pa), placed(i, *pb))

    # the slot, counterclockwise in (u, v)
    shell.face(bspline_surface(step, rows, u_knots, v_knots, 3),
               [(rim_left, True), (section1, True), (rim_right, False),
                (section0, False)])
    # the walls of the block: left (+y) with v up, right with v down, so
    # that du x dv is outward
    shell.face(bspline_surface(
        step, [[placed(i, w, -BASE), placed(i, w, top)]
               for i in range(len(PATH))], u_knots, [0.0, 0.0, 1.0, 1.0], 1),
        [(base_left, True), (lines['top_left-base_left1'], False),
         (top_left, False), (lines['top_left-base_left0'], True)])
    shell.face(bspline_surface(
        step, [[placed(i, -w, top), placed(i, -w, -BASE)]
               for i in range(len(PATH))], u_knots, [0.0, 0.0, 1.0, 1.0], 1),
        [(top_right, True), (lines['top_right-base_right1'], True),
         (base_right, False), (lines['top_right-base_right0'], False)])
    # the top, either side of the slot, its normal up (-z), and the base,
    # its normal down: counterclockwise seen from outside
    shell.plane((0.0, 0.0, -top), (0.0, 0.0, -1.0), (1.0, 0.0, 0.0),
                [(rim_left, True), (lines['slot_left-top_left1'], True),
                 (top_left, False), (lines['slot_left-top_left0'], False)])
    shell.plane((0.0, 0.0, -top), (0.0, 0.0, -1.0), (1.0, 0.0, 0.0),
                [(rim_right, False), (lines['slot_right-top_right0'], True),
                 (top_right, True), (lines['slot_right-top_right1'], False)])
    shell.plane((0.0, 0.0, BASE), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0),
                [(base_left, False), (lines['base_left-base_right0'], True),
                 (base_right, True), (lines['base_left-base_right1'], False)])
    # the ends, their normals back along the path at the start and on along
    # it at the end
    for end, i, sign in (('0', 0, -1.0), ('1', last, 1.0)):
        n = normals()[i]
        forward = (sign * n[1], -sign * n[0], 0.0)
        loop = [(section0 if end == '0' else section1, True),
                (lines['slot_right-top_right' + end], True),
                (lines['top_right-base_right' + end], True),
                (lines['base_left-base_right' + end], False),
                (lines['top_left-base_left' + end], False),
                (lines['slot_left-top_left' + end], False)]
        if sign > 0.0:
            loop = [(e, not forward_) for e, forward_ in reversed(loop)]
        shell.plane((PATH[i][0], PATH[i][1], 0.0), forward, (n[0], n[1], 0.0),
                    loop)
    return shell.solid(name)


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'examples', 'cad')
    write(os.path.join(directory, 'torus.step'), 'torus',
          'a torus of %g mm major and %g mm minor radius about z, in four '
          'faces. In millimeters. Written by scripts/make_channel_step.py' %
          (MAJOR, MINOR), [('torus', torus, (0.75, 0.62, 0.30))])
    write(os.path.join(directory, 'channel.step'), 'channel',
          'a block with a slot along a cubic B-spline path, the slot one '
          'B-spline face; its least radius of curvature is %.0f mm. In '
          'millimeters. Written by scripts/make_channel_step.py' %
          path_radius(), [('channel', channel, (0.55, 0.57, 0.60))])


if __name__ == '__main__':
    main()
