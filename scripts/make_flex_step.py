#!/usr/bin/env python3
"""Write the STEP files of the slider-crank with a flexible connecting rod
(examples/flex/slider_crank_flexible_rod.json):

examples/flex/connecting_rod.step: a steel rod 250 mm between the centres of
its bores (12 mm diameter), with round ends of 15 mm radius joined by a
shank 10 mm wide, 10 mm thick. The big end's bore is about the z axis
through the origin, the small end's about the z axis through x = 250 mm,
and the rod runs from z = -5 mm to z = 5 mm.

examples/flex/crank.step: a crank 60 mm between the centres of its round
ends (12 mm radius), 10 mm thick, about the same axes: its shaft at the
origin and its pin at x = 60 mm, below the rod (z = 5 mm to 15 mm).

examples/flex/slider.step: a block 40 mm long (x), 30 mm wide (y) and 30 mm
high (z), centred on the origin.

In millimeters. Each solid is a prism, as written by make_track_step.py.

Usage: make_flex_step.py [output directory] (default examples/flex)
"""

import math
import os
import sys

from make_track_step import circle_loop, extrude, polygon_loop, write

ROD_LENGTH = 250.0
ROD_END_RADIUS = 15.0
ROD_SHANK_HALF_WIDTH = 5.0
ROD_BORE_RADIUS = 6.0
ROD_THICKNESS = 10.0
CRANK_LENGTH = 60.0
CRANK_END_RADIUS = 12.0
CRANK_THICKNESS = 10.0
SLIDER_SIZE = (40.0, 30.0, 30.0)

X = (1.0, 0.0, 0.0)
Y = (0.0, 1.0, 0.0)


def rod_outline():
    """the rod's outline, counterclockwise: the shank's lower side, the small
    end's arc in two halves, the shank's upper side, the big end's arc in two
    halves (no arc over half a turn)"""
    r = ROD_END_RADIUS
    w = ROD_SHANK_HALF_WIDTH
    s = math.sqrt(r * r - w * w)  # where the shank meets an end
    L = ROD_LENGTH
    return [('line', (s, -w)),
            ('arc', (L - s, -w), (L, 0.0), True),
            ('arc', (L + r, 0.0), (L, 0.0), True),
            ('line', (L - s, w)),
            ('arc', (s, w), (0.0, 0.0), True),
            ('arc', (-r, 0.0), (0.0, 0.0), True)]


def rod(step, name):
    return extrude(step, name,
                   [rod_outline(),
                    circle_loop((0.0, 0.0), ROD_BORE_RADIUS, False),
                    circle_loop((ROD_LENGTH, 0.0), ROD_BORE_RADIUS, False)],
                   (0.0, 0.0, -ROD_THICKNESS / 2.0), X, Y, ROD_THICKNESS)


def crank(step, name):
    r = CRANK_END_RADIUS
    L = CRANK_LENGTH
    outline = [('line', (0.0, -r)),
               ('arc', (L, -r), (L, 0.0), True),
               ('line', (L, r)),
               ('arc', (0.0, r), (0.0, 0.0), True)]
    return extrude(step, name, [outline], (0.0, 0.0, ROD_THICKNESS / 2.0), X,
                   Y, CRANK_THICKNESS)


def slider(step, name):
    a, b, c = (d / 2.0 for d in SLIDER_SIZE)
    return extrude(step, name,
                   [polygon_loop([(-a, -b), (a, -b), (a, b), (-a, b)])],
                   (0.0, 0.0, -c), X, Y, 2.0 * c)


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'examples', 'flex')
    os.makedirs(directory, exist_ok=True)
    write(os.path.join(directory, 'connecting_rod.step'), 'connecting_rod',
          'a connecting rod %g mm between the centres of its bores of %g mm '
          'diameter, round ends of %g mm radius, a shank %g mm wide, %g mm '
          'thick, about z, the big end at the origin. In millimeters. '
          'Written by scripts/make_flex_step.py' %
          (ROD_LENGTH, 2 * ROD_BORE_RADIUS, ROD_END_RADIUS,
           2 * ROD_SHANK_HALF_WIDTH, ROD_THICKNESS),
          [('connecting_rod', rod, (0.55, 0.57, 0.60))])
    write(os.path.join(directory, 'crank.step'), 'crank',
          'a crank %g mm between the centres of its round ends of %g mm '
          'radius, %g mm thick, from z = %g mm, the shaft at the origin. In '
          'millimeters. Written by scripts/make_flex_step.py' %
          (CRANK_LENGTH, CRANK_END_RADIUS, CRANK_THICKNESS,
           ROD_THICKNESS / 2.0),
          [('crank', crank, (0.75, 0.62, 0.30))])
    write(os.path.join(directory, 'slider.step'), 'slider',
          'a block %g by %g by %g mm centred on the origin. In millimeters. '
          'Written by scripts/make_flex_step.py' % SLIDER_SIZE,
          [('slider', slider, (0.35, 0.45, 0.60))])


if __name__ == '__main__':
    main()
