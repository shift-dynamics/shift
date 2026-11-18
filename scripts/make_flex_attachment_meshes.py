#!/usr/bin/env python3
"""Write the triangle meshes (OBJ, in meters) of
examples/flex/attachments.json, a model that shows how the frames of
flexible bodies are tied to their finite element meshes:

link.obj: a steel link along x, 200 mm from the centre of the spherical
socket at its root (a cavity of 10 mm radius, open towards -x, centred on
the origin, in a boss of 16 mm radius) to the centre of the ball at its tip
(11 mm radius, at x = 200 mm), joined by a shank of 7 mm radius.

seat.obj: the fixed ball in the link's socket, 9.5 mm radius.

cup.obj: the weight hanging on the link's ball: a cup of 11.5 mm cavity
radius, open towards -x, centred on its origin, 20 mm in radius and 24 mm
long.

puck.obj: a disc of 10 mm radius, 8 mm thick, about x.

tube.obj: a steel tube along x, 120 mm long, bore 20 mm, outer diameter 32
mm, from x = 0.

plunger.obj: the plunger sliding in the tube: two lands of 9.5 mm radius,
8 mm long, 8 mm apart, on a stem of 8 mm radius, centred on its origin.

Each solid is a surface of revolution about x, its triangles closed and
facing out.

Usage: make_flex_attachment_meshes.py [output directory]
(default examples/flex)
"""

import math
import os
import sys

SIDES = 32
ARC = 12  # segments of a quarter circle


def arc(cx, radius, start, stop, count):
    """points (x, r) of the circle of radius about (cx, 0) from angle start
    to stop (from +x towards +r)"""
    return [(cx + radius * math.cos(start + (stop - start) * k / count),
             radius * math.sin(start + (stop - start) * k / count))
            for k in range(count + 1)]


def lathe(profile, closed=False):
    """vertices and triangles of the profile [(x, r), ...] turned about x:
    a profile from the axis to the axis, or a closed loop off the axis"""
    vertices = []
    rings = []
    for x, r in profile:
        if r < 1e-12:
            rings.append([len(vertices)])
            vertices.append((x, 0.0, 0.0))
        else:
            ring = []
            for i in range(SIDES):
                a = 2.0 * math.pi * i / SIDES
                ring.append(len(vertices))
                vertices.append((x, r * math.cos(a), r * math.sin(a)))
            rings.append(ring)
    triangles = []
    count = len(rings) if closed else len(rings) - 1
    for k in range(count):
        p, q = rings[k], rings[(k + 1) % len(rings)]
        for i in range(SIDES):
            j = (i + 1) % SIDES
            if len(p) == 1:
                triangles.append((p[0], q[i], q[j]))
            elif len(q) == 1:
                triangles.append((p[i], q[0], p[j]))
            else:
                triangles.append((p[i], q[i], q[j]))
                triangles.append((p[i], q[j], p[j]))
    # facing out: the signed volume is positive
    volume = 0.0
    for a, b, c in triangles:
        u, v, w = vertices[a], vertices[b], vertices[c]
        volume += (u[0] * (v[1] * w[2] - v[2] * w[1]) -
                   u[1] * (v[0] * w[2] - v[2] * w[0]) +
                   u[2] * (v[0] * w[1] - v[1] * w[0])) / 6.0
    if volume < 0.0:
        triangles = [(a, c, b) for a, b, c in triangles]
    return vertices, triangles


def write(path, mesh):
    vertices, triangles = mesh
    with open(path, 'w') as f:
        for v in vertices:
            f.write('v %.9g %.9g %.9g\n' % v)
        for t in triangles:
            f.write('f %d %d %d\n' % (t[0] + 1, t[1] + 1, t[2] + 1))


def cup(cavity, radius, length):
    """a cup open towards -x, its cavity a hemisphere about the origin"""
    return (arc(0.0, cavity, 0.0, math.pi / 2, ARC) +
            [(0.0, radius), (length, radius), (length, 0.0)])


def link():
    cavity, boss, boss_length = 0.010, 0.016, 0.022
    shank, ball, length = 0.007, 0.011, 0.200
    join = math.pi - math.asin(shank / ball)
    return (arc(0.0, cavity, 0.0, math.pi / 2, ARC) +
            [(0.0, boss), (boss_length, boss), (boss_length, shank)] +
            arc(length, ball, join, 0.0, 2 * ARC))


def plunger():
    stem, land = 0.008, 0.0095
    lands = [(-0.012, -0.004), (0.004, 0.012)]
    profile = [(-0.012, 0.0)]
    for a, b in lands:
        profile += [(a, stem), (a, land), (b, land), (b, stem)]
    profile.append((0.012, 0.0))
    return profile


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'examples', 'flex')
    write(os.path.join(out, 'link.obj'), lathe(link()))
    write(os.path.join(out, 'seat.obj'),
          lathe(arc(0.0, 0.0095, 0.0, math.pi, 2 * ARC)))
    write(os.path.join(out, 'cup.obj'), lathe(cup(0.0115, 0.020, 0.024)))
    write(os.path.join(out, 'puck.obj'),
          lathe([(-0.004, 0.0), (-0.004, 0.010), (0.004, 0.010),
                 (0.004, 0.0)]))
    write(os.path.join(out, 'tube.obj'),
          lathe([(0.0, 0.010), (0.0, 0.016), (0.120, 0.016), (0.120, 0.010)],
                closed=True))
    write(os.path.join(out, 'plunger.obj'), lathe(plunger()))


if __name__ == '__main__':
    main()
