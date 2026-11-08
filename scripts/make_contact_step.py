#!/usr/bin/env python3
"""Write the STEP files of the contact tests that need exact shapes:
examples/cad/gear_tooth.step, one tooth of a spur gear, and ring_sector.step, a
quarter of a bent tube.

The gear has 20 teeth of module 5 mm and a pressure angle of 20 degrees, so a
pitch radius of 50 mm, and is 20 mm wide. The tooth lies along x from the
centre of the gear at the origin, and two of them on gears 100 mm apart mesh with
a line of contact, along the line of action. Each flank is an involute, a cubic
B-spline fitted to it to within a micrometre, extruded along z.

The ring is the part of a tube of radius 8 mm, bent to a radius of 30 mm about z,
facing +z and within 45 degrees of it, over a quarter of a turn from x to y: one
exact rational surface, touching a plane z = 8 mm along a quarter circle.

Usage: make_contact_step.py [output directory] (default examples/cad)
"""

import math
import os
import sys

import numpy

from make_cam_step import WEIGHTS, rational_curve
from make_track_step import write

MODULE = 5.0
TEETH = 20
PRESSURE_ANGLE = math.radians(20.0)
WIDTH = 20.0
RING_RADIUS = 30.0
TUBE_RADIUS = 8.0
SPLINE_POINTS = 12


def involute(rb, roll):
    return (rb * (math.cos(roll) + roll * math.sin(roll)),
            rb * (math.sin(roll) - roll * math.cos(roll)))


def basis(knots, degree, count, t):
    """the cubic B-spline basis functions at t, Cox and de Boor"""
    values = [1.0 if knots[i] <= t < knots[i + 1] else 0.0
              for i in range(len(knots) - 1)]
    if t >= knots[-1]:
        values[count - 1] = 1.0
    for d in range(1, degree + 1):
        for i in range(len(knots) - 1 - d):
            left = knots[i + d] - knots[i]
            right = knots[i + d + 1] - knots[i + 1]
            a = (t - knots[i]) / left * values[i] if left > 0.0 else 0.0
            b = ((knots[i + d + 1] - t) / right * values[i + 1]
                 if right > 0.0 else 0.0)
            values[i] = a + b
    return values[:count]


def flank():
    """control points and knots of the upper flank, from the base circle to the
    tip, the tooth centred on x"""
    rp = MODULE * TEETH / 2.0
    rb = rp * math.cos(PRESSURE_ANGLE)
    ra = rp + MODULE
    # the flank at the pitch circle is a quarter of a pitch from the centreline
    pitch_roll = math.tan(PRESSURE_ANGLE)
    x, y = involute(rb, pitch_roll)
    turn = math.pi / (2.0 * TEETH) + math.atan2(y, x)
    roll_tip = math.sqrt((ra / rb) ** 2 - 1.0)
    spans = SPLINE_POINTS - 3
    knots = [0.0] * 4 + [i / spans for i in range(1, spans)] + [1.0] * 4
    samples = numpy.linspace(0.0, 1.0, 400)
    target = []
    for t in samples:
        px, py = involute(rb, roll_tip * t)
        c, s = math.cos(turn), math.sin(turn)
        # the involute unwinds clockwise from the tooth's upper side
        target.append((c * px + s * py, s * px - c * py))
    target = numpy.array(target)
    matrix = numpy.array([basis(knots, 3, SPLINE_POINTS, t) for t in samples])
    points, *_ = numpy.linalg.lstsq(matrix, target, rcond=None)
    points[0], points[-1] = target[0], target[-1]
    error = numpy.abs(matrix @ points - target).max()
    assert error < 1e-3, error  # millimetres
    return [(float(p[0]), float(p[1])) for p in points], knots


def spline_curve(step, points, knots):
    distinct = sorted(set(knots))
    return step.add(
        "B_SPLINE_CURVE_WITH_KNOTS('',3,(%s),.UNSPECIFIED.,.F.,.F.,(%s),(%s),"
        ".UNSPECIFIED.)" %
        (','.join('#%d' % step.point(p) for p in points),
         ','.join(str(knots.count(k)) for k in distinct),
         ','.join(step.num(k) for k in distinct)))


def prism(step, name, segments):
    """a solid of a closed outline, counterclockwise from +z, extruded from z =
    0 to WIDTH: each segment ('line', p, q), or ('spline', points, knots)"""
    z_axis = (0.0, 0.0, 1.0)
    starts = [s[1] if s[0] == 'line' else s[1][0] for s in segments]
    count = len(segments)
    bottom_v = [step.add("VERTEX_POINT('',#%d)" %
                         step.point((p[0], p[1], 0.0))) for p in starts]
    top_v = [step.add("VERTEX_POINT('',#%d)" %
                      step.point((p[0], p[1], WIDTH))) for p in starts]
    up = []
    for i, p in enumerate(starts):
        line = step.add("LINE('',#%d,#%d)" % (
            step.point((p[0], p[1], 0.0)),
            step.add("VECTOR('',#%d,%s)" % (step.direction(z_axis),
                                            step.num(WIDTH)))))
        up.append(step.add("EDGE_CURVE('',#%d,#%d,#%d,.T.)" %
                           (bottom_v[i], top_v[i], line)))

    def edge(segment, height, start, end):
        if segment[0] == 'line':
            p, q = segment[1], segment[2]
            length = math.hypot(q[0] - p[0], q[1] - p[1])
            curve = step.add("LINE('',#%d,#%d)" % (
                step.point((p[0], p[1], height)),
                step.add("VECTOR('',#%d,%s)" % (
                    step.direction(((q[0] - p[0]) / length,
                                    (q[1] - p[1]) / length, 0.0)),
                    step.num(length)))))
        else:
            curve = spline_curve(step, [(p[0], p[1], height)
                                        for p in segment[1]], segment[2])
        return curve, step.add("EDGE_CURVE('',#%d,#%d,#%d,.T.)" %
                               (start, end, curve))

    faces, bottom, top = [], [], []
    for i, segment in enumerate(segments):
        j = (i + 1) % count
        low, bottom_edge = edge(segment, 0.0, bottom_v[i], bottom_v[j])
        _, top_edge = edge(segment, WIDTH, top_v[i], top_v[j])
        bottom.append(bottom_edge)
        top.append(top_edge)
        if segment[0] == 'line':
            p, q = segment[1], segment[2]
            length = math.hypot(q[0] - p[0], q[1] - p[1])
            along = ((q[0] - p[0]) / length, (q[1] - p[1]) / length, 0.0)
            surface = step.add("PLANE('',#%d)" % step.placement(
                (p[0], p[1], 0.0), (along[1], -along[0], 0.0), along))
        else:
            surface = step.add(
                "SURFACE_OF_LINEAR_EXTRUSION('',#%d,#%d)" %
                (low, step.add("VECTOR('',#%d,%s)" % (step.direction(z_axis),
                                                      step.num(WIDTH)))))
        oriented = [step.add("ORIENTED_EDGE('',*,*,#%d,.T.)" % bottom_edge),
                    step.add("ORIENTED_EDGE('',*,*,#%d,.T.)" % up[j]),
                    step.add("ORIENTED_EDGE('',*,*,#%d,.F.)" % top_edge),
                    step.add("ORIENTED_EDGE('',*,*,#%d,.F.)" % up[i])]
        loop = step.add("EDGE_LOOP('',(%s))" %
                        ','.join('#%d' % e for e in oriented))
        bound = step.add("FACE_OUTER_BOUND('',#%d,.T.)" % loop)
        faces.append(step.add("ADVANCED_FACE('',(#%d),#%d,.T.)" %
                              (bound, surface)))
    for name_, edges, sense, z, normal in (
            ('top', top, '.T.', WIDTH, z_axis),
            ('bottom', list(reversed(bottom)), '.F.', 0.0, (0.0, 0.0, -1.0))):
        loop = step.add("EDGE_LOOP('',(%s))" % ','.join(
            '#%d' % step.add("ORIENTED_EDGE('',*,*,#%d,%s)" % (e, sense))
            for e in edges))
        bound = step.add("FACE_OUTER_BOUND('',#%d,.T.)" % loop)
        plane = step.add("PLANE('',#%d)" % step.placement(
            (0.0, 0.0, z), normal, (1.0, 0.0, 0.0)))
        faces.append(step.add("ADVANCED_FACE('%s',(#%d),#%d,.T.)" %
                              (name_, bound, plane)))
    shell = step.add("CLOSED_SHELL('',(%s))" %
                     ','.join('#%d' % f for f in faces))
    return step.add("MANIFOLD_SOLID_BREP('%s',#%d)" % (name, shell))


def tooth(step, name):
    rp = MODULE * TEETH / 2.0
    root = rp - 1.25 * MODULE
    rb = rp * math.cos(PRESSURE_ANGLE)
    points, knots = flank()
    lower = [(p[0], -p[1]) for p in points]
    upper = list(reversed(points))
    base = points[0]
    root_up = (root * base[0] / rb, root * base[1] / rb)
    root_down = (root_up[0], -root_up[1])
    flip = [1.0 - k for k in reversed(knots)]
    return prism(step, name, [
        ('line', root_down, lower[0]),
        ('spline', lower, knots),
        ('line', lower[-1], upper[0]),
        ('spline', upper, flip),
        ('line', upper[-1], root_up),
        ('line', root_up, root_down)])


def ring(step, name):
    """the bent tube, one rational biquadratic face"""
    c, r, big = math.sqrt(0.5), TUBE_RADIUS, RING_RADIUS
    section = [(big + r * c, r * c), (big, r * 2.0 * c), (big - r * c, r * c)]
    around = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    grid = [[(a[0] * s[0], a[1] * s[0], s[1]) for s in section]
            for a in around]
    rows = ','.join('(%s)' % ','.join('#%d' % step.point(p) for p in row)
                    for row in grid)
    weights = ','.join('(%s)' % ','.join(repr(a * b) for b in WEIGHTS)
                       for a in WEIGHTS)
    surface = step.add(
        "(BOUNDED_SURFACE()B_SPLINE_SURFACE(2,2,(%s),.UNSPECIFIED.,.F.,.F.,"
        ".F.)B_SPLINE_SURFACE_WITH_KNOTS((3,3),(3,3),(0.,1.),(0.,1.),"
        ".UNSPECIFIED.)GEOMETRIC_REPRESENTATION_ITEM()"
        "RATIONAL_B_SPLINE_SURFACE((%s))REPRESENTATION_ITEM('')SURFACE())" %
        (rows, weights))
    corners = [grid[0][0], grid[2][0], grid[2][2], grid[0][2]]
    vertices = [step.add("VERTEX_POINT('',#%d)" % step.point(p))
                for p in corners]
    sides = [[grid[i][0] for i in range(3)],
             grid[2],
             [grid[i][2] for i in (2, 1, 0)],
             list(reversed(grid[0]))]
    oriented = []
    for i, side in enumerate(sides):
        edge = step.add("EDGE_CURVE('',#%d,#%d,#%d,.T.)" % (
            vertices[i], vertices[(i + 1) % 4], rational_curve(step, side)))
        oriented.append(step.add("ORIENTED_EDGE('',*,*,#%d,.T.)" % edge))
    loop = step.add("EDGE_LOOP('',(%s))" % ','.join('#%d' % e
                                                    for e in oriented))
    bound = step.add("FACE_OUTER_BOUND('',#%d,.T.)" % loop)
    face = step.add("ADVANCED_FACE('',(#%d),#%d,.T.)" % (bound, surface))
    shell = step.add("CLOSED_SHELL('',(#%d))" % face)
    return step.add("MANIFOLD_SOLID_BREP('%s',#%d)" % (name, shell))


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else 'examples/cad'
    steel = (0.6, 0.62, 0.65)
    write(os.path.join(directory, 'gear_tooth.step'), 'gear_tooth',
          'a tooth of a spur gear, see scripts/make_contact_step.py',
          [('tooth', tooth, steel)])
    write(os.path.join(directory, 'ring_sector.step'), 'ring_sector',
          'a quarter of a bent tube, see scripts/make_contact_step.py',
          [('ring', ring, steel)])


if __name__ == '__main__':
    main()
