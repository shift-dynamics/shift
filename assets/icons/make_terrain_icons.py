#!/usr/bin/env python
"""Draw the icons of the terrain types as line art, 512 x 512, black on
transparent, as the other icons:

heightmap.png  a heightmap ("basic"): a bicubic surface patch, one hill
scm.png        deformable soil ("scm"): a wheel sunk in a rut in the soil
sph.png        SPH soil ("sph"): a heap of particles
"""
import math

import cairo

SIZE = 512
LINE = 28.0  # as the strokes of terrain.png


def Canvas():
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, SIZE, SIZE)
    c = cairo.Context(surface)
    c.set_source_rgba(0, 0, 0, 1)
    c.set_line_cap(cairo.LINE_CAP_ROUND)
    c.set_line_join(cairo.LINE_JOIN_ROUND)
    return surface, c


def Heightmap(path):
    """a patch z = h(u, v) over the unit square, seen from above at an angle;
    its edges bold, a 3 x 3 grid of its parameter lines inside"""
    surface, c = Canvas()

    def height(u, v):
        # a broad hill, off center, that lifts the edges near it
        return 0.32 * math.exp(-((u - 0.4) ** 2 + (v - 0.45) ** 2) / 0.12)

    def project(u, v):
        x = (u - v) * math.cos(math.radians(30))
        y = (u + v) * math.sin(math.radians(30)) - height(u, v)
        return x, y

    # fit the patch to the canvas
    points = [project(i / 40, j / 40) for i in range(41) for j in range(41)]
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    margin = LINE
    scale = (SIZE - 2 * margin) / max(max(xs) - min(xs), max(ys) - min(ys))
    x0 = SIZE / 2 - scale * (max(xs) + min(xs)) / 2
    y0 = SIZE / 2 - scale * (max(ys) + min(ys)) / 2

    def curve(points, width):
        c.set_line_width(width)
        for k, (x, y) in enumerate(points):
            (c.move_to if k == 0 else c.line_to)(x0 + scale * x, y0 + scale * y)
        c.stroke()

    n = 60
    for line in range(1, 3):
        t = line / 3
        curve([project(t, s / n) for s in range(n + 1)], 0.6 * LINE)
        curve([project(s / n, t) for s in range(n + 1)], 0.6 * LINE)
    # the edges, as one closed curve
    edge = ([project(s / n, 0) for s in range(n + 1)] +
            [project(1, s / n) for s in range(n + 1)] +
            [project(1 - s / n, 1) for s in range(n + 1)] +
            [project(0, 1 - s / n) for s in range(n + 1)])
    c.set_line_width(LINE)
    for k, (x, y) in enumerate(edge):
        (c.move_to if k == 0 else c.line_to)(x0 + scale * x, y0 + scale * y)
    c.close_path()
    c.stroke()
    surface.write_to_png(path)


def Scm(path):
    """a wheel sunk in the soil, which is pushed up beside it; the soil
    hatched below its surface"""
    surface, c = Canvas()
    r = 120.0
    cx, cy = SIZE / 2, 250.0
    ground = 330.0
    # the surface: flat, a berm up, into the rut under the wheel, a berm, flat
    sink = cy + r - LINE / 2  # the rut follows the wheel's rim
    c.set_line_width(LINE)
    c.move_to(LINE, ground)
    c.line_to(cx - r - 60, ground)
    c.curve_to(cx - r - 30, ground, cx - r - 25, ground - 34,
               cx - r + 4, ground - 30)
    # down along the rim
    a0 = math.acos((ground - 30 - cy) / (r + LINE / 2))
    c.arc_negative(cx, cy, r + LINE / 2, math.pi / 2 + a0, math.pi / 2 - a0)
    c.curve_to(cx + r + 25, ground - 34, cx + r + 30, ground,
               cx + r + 60, ground)
    c.line_to(SIZE - LINE, ground)
    c.stroke()
    del sink
    # the wheel and its hub
    c.arc(cx, cy, r - LINE / 2, 0, 2 * math.pi)
    c.stroke()
    c.arc(cx, cy, 26, 0, 2 * math.pi)
    c.fill()
    # the soil below
    c.set_line_width(0.6 * LINE)
    for k in range(7):
        x = LINE + 20 + k * 72
        c.move_to(x, 410)
        c.line_to(x - 44, 470)
        c.stroke()
    surface.write_to_png(path)


def Sph(path):
    """a heap of particles: rows of circles, packed, fewer up the heap"""
    surface, c = Canvas()
    rows = [4, 3, 2, 1]
    width = SIZE - 2 * LINE
    d = width / rows[0]  # the diameter, the circles touching
    r = d / 2 - LINE * 0.6
    c.set_line_width(LINE)
    bottom = SIZE - LINE - d / 2
    for level, count in enumerate(rows):
        y = bottom - level * d * math.sqrt(3) / 2
        x_first = SIZE / 2 - (count - 1) * d / 2
        for k in range(count):
            c.arc(x_first + k * d, y, r, 0, 2 * math.pi)
            c.stroke()
    surface.write_to_png(path)


if __name__ == "__main__":
    Heightmap("heightmap.png")
    Scm("scm.png")
    Sph("sph.png")
