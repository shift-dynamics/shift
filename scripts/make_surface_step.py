#!/usr/bin/env python3
"""Write the STEP files of the surfaces that the other examples do not have,
for the theory pages on contact (doc/sphinx/contact):
examples/cad/torus_patch.step, a quarter of the outer half of a torus, and
revolution_patch.step, a quarter turn of a B-spline profile about an axis.

The torus has a major radius of 30 mm and a minor radius of 10 mm about z, and
the patch runs from x to y around z (u from 0 to 90 degrees) and over the outer
half of the tube (v from -90 to 90 degrees), bounded by two circles about z
and two circles of the tube.

The surface of revolution turns a cubic B-spline profile in the xz plane, a
vase 40 mm tall, about z from x to y. It is bounded by the profile, the profile
turned by 90 degrees, and the circles its ends sweep.

Each is one face, written as a solid of one face as ring_sector.step is.

Usage: make_surface_step.py [output directory] (default examples/cad)
"""

import math
import os
import sys

from make_contact_step import spline_curve
from make_track_step import write

MAJOR = 30.0
MINOR = 10.0
PROFILE = [(20.0, 0.0), (28.0, 8.0), (16.0, 20.0), (10.0, 30.0), (16.0, 40.0)]
PROFILE_KNOTS = [0.0] * 4 + [0.5] + [1.0] * 4


def circle(step, center, axis, start, radius):
    """a circle of radius about axis through center, its parameter 0 along
    start"""
    return step.add("CIRCLE('',#%d,%s)" % (
        step.placement(center, axis, start), step.num(radius)))


def face_solid(step, name, surface, corners, sides):
    """a solid of one face on surface, bounded by the sides between the
    corners in turn: each (curve, forward), forward if the curve runs from
    this corner to the next"""
    vertices = [step.add("VERTEX_POINT('',#%d)" % step.point(p))
                for p in corners]
    oriented = []
    for i, (curve, forward) in enumerate(sides):
        a, b = vertices[i], vertices[(i + 1) % len(vertices)]
        edge = step.add("EDGE_CURVE('',#%d,#%d,#%d,.T.)" %
                        ((a, b, curve) if forward else (b, a, curve)))
        oriented.append(step.add("ORIENTED_EDGE('',*,*,#%d,%s)" %
                                 (edge, '.T.' if forward else '.F.')))
    loop = step.add("EDGE_LOOP('',(%s))" % ','.join('#%d' % e
                                                    for e in oriented))
    bound = step.add("FACE_OUTER_BOUND('',#%d,.T.)" % loop)
    face = step.add("ADVANCED_FACE('',(#%d),#%d,.T.)" % (bound, surface))
    shell = step.add("CLOSED_SHELL('',(#%d))" % face)
    return step.add("MANIFOLD_SOLID_BREP('%s',#%d)" % (name, shell))


def torus(step, name):
    """(u, v) from (0, -90) to (90, 90) degrees, counterclockwise in (u, v),
    so that the face's normal is the torus's outward one"""
    surface = step.add("TOROIDAL_SURFACE('',#%d,%s,%s)" % (
        step.placement((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
        step.num(MAJOR), step.num(MINOR)))
    corners = [(MAJOR, 0.0, -MINOR), (0.0, MAJOR, -MINOR),
               (0.0, MAJOR, MINOR), (MAJOR, 0.0, MINOR)]
    sides = [
        # v = -90: about z from x to y
        (circle(step, (0.0, 0.0, -MINOR), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0),
                MAJOR), True),
        # u = 90: the tube's circle in the yz plane, from below to above
        (circle(step, (0.0, MAJOR, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                MINOR), True),
        # v = 90: about z, run from y back to x
        (circle(step, (0.0, 0.0, MINOR), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0),
                MAJOR), False),
        # u = 0: the tube's circle in the xz plane, run from above to below
        (circle(step, (MAJOR, 0.0, 0.0), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0),
                MINOR), False),
    ]
    return face_solid(step, name, surface, corners, sides)


def revolution(step, name):
    """u the angle about z from 0 to 90 degrees, v the profile's parameter
    from 0 to 1"""
    profile = spline_curve(step, [(r, 0.0, z) for r, z in PROFILE],
                           PROFILE_KNOTS)
    turned = spline_curve(step, [(0.0, r, z) for r, z in PROFILE],
                          PROFILE_KNOTS)
    surface = step.add("SURFACE_OF_REVOLUTION('',#%d,#%d)" % (
        profile, step.add("AXIS1_PLACEMENT('',#%d,#%d)" % (
            step.point((0.0, 0.0, 0.0)), step.direction((0.0, 0.0, 1.0))))))
    (r0, z0), (r1, z1) = PROFILE[0], PROFILE[-1]
    corners = [(r0, 0.0, z0), (0.0, r0, z0), (0.0, r1, z1), (r1, 0.0, z1)]
    sides = [
        (circle(step, (0.0, 0.0, z0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0), r0),
         True),
        (turned, True),
        (circle(step, (0.0, 0.0, z1), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0), r1),
         False),
        (profile, False),
    ]
    return face_solid(step, name, surface, corners, sides)


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else 'examples/cad'
    steel = (0.6, 0.62, 0.65)
    write(os.path.join(directory, 'torus_patch.step'), 'torus_patch',
          'a patch of a torus, see scripts/make_surface_step.py',
          [('torus', torus, steel)])
    write(os.path.join(directory, 'revolution_patch.step'), 'revolution_patch',
          'a patch of a surface of revolution, see scripts/make_surface_step.py',
          [('revolution', revolution, steel)])


if __name__ == '__main__':
    main()
