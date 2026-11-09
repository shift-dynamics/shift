#!/usr/bin/env python3
"""Write the STEP files of a peg and a plate with a hole, for the tests of the
insertion of a peg on the exact surfaces of STEP parts
(test/contact/test_cad_contact.cc):

examples/cad/peg.step: a shaft of 20 mm diameter and 50 mm length under a head
of 32 mm diameter and 8 mm height, coaxial about z. The shaft runs from z = 0
to z = 50 mm and the head from z = -8 mm to z = 0, so that the origin is at the
center of the underside of the head.

examples/cad/plate_hole.step: a plate 80 mm square and 20 mm thick, from z = 0
to z = 20 mm, with a hole of 21 mm diameter through it about z: 0.5 mm of
radial clearance about the shaft.

In millimeters. Each solid is a prism, as written by make_track_step.py.

Usage: make_peg_step.py [output directory] (default examples/cad)
"""

import os
import sys

from make_track_step import circle_loop, extrude, polygon_loop, write

SHAFT_RADIUS = 10.0
SHAFT_LENGTH = 50.0
HEAD_RADIUS = 16.0
HEAD_HEIGHT = 8.0
HOLE_RADIUS = 10.5
PLATE_HALF_WIDTH = 40.0
PLATE_THICKNESS = 20.0

X = (1.0, 0.0, 0.0)
Y = (0.0, 1.0, 0.0)


def shaft(step, name):
    return extrude(step, name, [circle_loop((0.0, 0.0), SHAFT_RADIUS, True)],
                   (0.0, 0.0, 0.0), X, Y, SHAFT_LENGTH)


def head(step, name):
    return extrude(step, name, [circle_loop((0.0, 0.0), HEAD_RADIUS, True)],
                   (0.0, 0.0, -HEAD_HEIGHT), X, Y, HEAD_HEIGHT)


def plate(step, name):
    w = PLATE_HALF_WIDTH
    return extrude(step, name,
                   [polygon_loop([(-w, -w), (w, -w), (w, w), (-w, w)]),
                    circle_loop((0.0, 0.0), HOLE_RADIUS, False)],
                   (0.0, 0.0, 0.0), X, Y, PLATE_THICKNESS)


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'examples', 'cad')
    write(os.path.join(directory, 'peg.step'), 'peg',
          'a peg: a shaft of %g mm diameter, %g mm long, under a head of %g mm '
          'diameter, %g mm high, about z, the origin at the underside of the '
          'head. In millimeters. Written by scripts/make_peg_step.py' %
          (2 * SHAFT_RADIUS, SHAFT_LENGTH, 2 * HEAD_RADIUS, HEAD_HEIGHT),
          [('shaft', shaft, (0.75, 0.62, 0.30)),
           ('head', head, (0.75, 0.62, 0.30))])
    write(os.path.join(directory, 'plate_hole.step'), 'plate_hole',
          'a plate %g mm square, %g mm thick, from z = 0 to z = %g mm, with a '
          'hole of %g mm diameter through it about z. In millimeters. Written '
          'by scripts/make_peg_step.py' %
          (2 * PLATE_HALF_WIDTH, PLATE_THICKNESS, PLATE_THICKNESS,
           2 * HOLE_RADIUS),
          [('plate', plate, (0.55, 0.57, 0.60))])


if __name__ == '__main__':
    main()
