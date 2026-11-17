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

examples/flex/slider.step: a piston sliding along x, its wrist pin about
the z axis through the origin: a cylindrical crown of 20 mm radius about x
from x = 18 mm to 40 mm, clear of the rod's small end (15 mm radius); two pin
bosses, cheeks 8 mm thick above and below the rod (7 mm to 15 mm from the
rod's mid plane, the rod being 10 mm thick), 20 mm wide, round about the pin
and joined to the crown; and the wrist pin, 11 mm in diameter (the rod's
bore is 12 mm), through them.

In millimeters. Each solid is a prism, as written by make_track_step.py.

Usage: make_flex_step.py [output directory] (default examples/flex)
"""

import math
import os
import sys

from make_track_step import circle_loop, extrude, write

ROD_LENGTH = 250.0
ROD_END_RADIUS = 15.0
ROD_SHANK_HALF_WIDTH = 5.0
ROD_BORE_RADIUS = 6.0
ROD_THICKNESS = 10.0
CRANK_LENGTH = 60.0
CRANK_END_RADIUS = 12.0
CRANK_THICKNESS = 10.0
PISTON_RADIUS = 20.0
PISTON_CROWN = (18.0, 40.0)  # x from, to: clear of the rod's small end
PISTON_BOSS_HALF_WIDTH = 10.0
PISTON_BOSS_Z = (7.0, 15.0)  # |z| from, to: clear of the rod (5 mm)
PISTON_PIN_RADIUS = 5.5

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


Z = (0.0, 0.0, 1.0)


def piston_crown(step, name):
    x0, x1 = PISTON_CROWN
    return extrude(step, name,
                   [circle_loop((0.0, 0.0), PISTON_RADIUS, True)],
                   (x0, 0.0, 0.0), Y, Z, x1 - x0)


def piston_boss(z0):
    """a pin boss from z0, 8 mm along z: round about the pin, reaching
    1 mm into the crown, with the pin's hole"""
    w = PISTON_BOSS_HALF_WIDTH
    x = PISTON_CROWN[0] + 1.0
    outline = [('line', (0.0, -w)),
               ('line', (x, -w)),
               ('line', (x, w)),
               ('arc', (0.0, w), (0.0, 0.0), True)]

    def build(step, name):
        return extrude(step, name,
                       [outline,
                        circle_loop((0.0, 0.0), PISTON_PIN_RADIUS, False)],
                       (0.0, 0.0, z0), X, Y,
                       PISTON_BOSS_Z[1] - PISTON_BOSS_Z[0])
    return build


def piston_pin(step, name):
    z = PISTON_BOSS_Z[1]
    return extrude(step, name,
                   [circle_loop((0.0, 0.0), PISTON_PIN_RADIUS, True)],
                   (0.0, 0.0, -z), X, Y, 2.0 * z)


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
    piston = (0.35, 0.45, 0.60)
    write(os.path.join(directory, 'slider.step'), 'slider',
          'a piston of %g mm radius sliding along x, its crown from x = %g '
          'to %g mm, its wrist pin of %g mm diameter about the z axis '
          'through the origin in bosses %g to %g mm either side of z = 0. In '
          'millimeters. Written by scripts/make_flex_step.py' %
          (PISTON_RADIUS, PISTON_CROWN[0], PISTON_CROWN[1],
           2 * PISTON_PIN_RADIUS, PISTON_BOSS_Z[0], PISTON_BOSS_Z[1]),
          [('crown', piston_crown, piston),
           ('boss_lower', piston_boss(PISTON_BOSS_Z[0]), piston),
           ('boss_upper', piston_boss(-PISTON_BOSS_Z[1]), piston),
           ('wrist_pin', piston_pin, (0.62, 0.64, 0.68))])


if __name__ == '__main__':
    main()
