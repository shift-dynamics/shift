#!/usr/bin/env python3
"""Write the schematic figures of doc/sphinx/flexible-bodies (SVG): how
the frames of a flexible body are tied to its finite element mesh for each
type of joint, and the boundary conditions of its reduction.

The colors are the viewer's (shift::gui::kAttachmentColors): the reference
frame's attachment red, the others orange, blue, green, ...

Usage: make_flex_attachment_figures.py [output directory]
(default doc/sphinx/images/flex)
"""

import math
import os
import sys

RED = '#e61a1a'
ORANGE = '#ff8c1a'
BLUE = '#338cff'
GREEN = '#40cc4d'
PURPLE = '#b366f2'
BODY = '#d5dae1'
EDGE = '#4a5058'
TEXT = '#1d2125'
MUTED = '#5f6670'
FONT = ('font-family="Helvetica, Arial, sans-serif"')


class Svg:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.items = []

    def add(self, item):
        self.items.append(item)

    def text(self, x, y, lines, size=13, weight='normal', color=TEXT,
             anchor='start'):
        if isinstance(lines, str):
            lines = [lines]
        spans = ''.join(
            '<tspan x="%.1f" dy="%s">%s</tspan>' %
            (x, '0' if i == 0 else '1.35em', escape(line))
            for i, line in enumerate(lines))
        self.add('<text x="%.1f" y="%.1f" font-size="%d" font-weight="%s" '
                 'fill="%s" text-anchor="%s" %s>%s</text>' %
                 (x, y, size, weight, color, anchor, FONT, spans))

    def line(self, x0, y0, x1, y1, color=EDGE, width=1.0, opacity=1.0,
             dash=None):
        self.add('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" '
                 'stroke="%s" stroke-width="%.2f" stroke-opacity="%.2f"%s/>'
                 % (x0, y0, x1, y1, color, width, opacity,
                    ' stroke-dasharray="%s"' % dash if dash else ''))

    def circle(self, x, y, r, fill='none', stroke=EDGE, width=1.0,
               dash=None, opacity=1.0):
        self.add('<circle cx="%.1f" cy="%.1f" r="%.1f" fill="%s" '
                 'stroke="%s" stroke-width="%.2f" fill-opacity="%.2f"%s/>'
                 % (x, y, r, fill, stroke, width, opacity,
                    ' stroke-dasharray="%s"' % dash if dash else ''))

    def path(self, d, fill='none', stroke=EDGE, width=1.2, dash=None,
             opacity=1.0, rule='nonzero'):
        self.add('<path d="%s" fill="%s" stroke="%s" stroke-width="%.2f" '
                 'fill-opacity="%.2f" fill-rule="%s"%s/>'
                 % (d, fill, stroke, width, opacity, rule,
                    ' stroke-dasharray="%s"' % dash if dash else ''))

    def rect(self, x, y, w, h, fill=BODY, stroke=EDGE, width=1.2, rx=0):
        self.add('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" '
                 'rx="%.1f" fill="%s" stroke="%s" stroke-width="%.2f"/>'
                 % (x, y, w, h, rx, fill, stroke, width))

    def write(self, path):
        with open(path, 'w') as f:
            f.write('<svg xmlns="http://www.w3.org/2000/svg" '
                    'width="%d" height="%d" viewBox="0 0 %d %d">\n' %
                    (self.width, self.height, self.width, self.height))
            f.write('<rect width="100%" height="100%" fill="#ffffff"/>\n')
            f.write(ARROW_DEFS)
            for item in self.items:
                f.write(item + '\n')
            f.write('</svg>\n')


ARROW_DEFS = '''<defs>
<marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7"
 markerHeight="7" orient="auto-start-reverse">
<path d="M 0 0 L 10 5 L 0 10 z" fill="#1d2125"/></marker>
</defs>
'''


def escape(s):
    return s.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def frame_marker(svg, x, y, label=None, dx=10, dy=-10):
    """the frame tied: a small target"""
    svg.circle(x, y, 6, fill='#ffffff', stroke=TEXT, width=1.6)
    svg.circle(x, y, 2, fill=TEXT, stroke=TEXT, width=1)
    if label:
        svg.text(x + dx, y + dy, label, size=12, weight='bold')


def spider(svg, x, y, nodes, color, width=0.9, opacity=0.6, dots=True):
    for nx, ny in nodes:
        svg.line(x, y, nx, ny, color=color, width=width, opacity=opacity)
    if dots:
        for nx, ny in nodes:
            svg.circle(nx, ny, 3.2, fill=color, stroke=color, width=0.5)


def arc_arrow(svg, x, y, r, start, stop, color=TEXT):
    """an arc about (x, y) from angle start to stop (degrees, y down)"""
    a0, a1 = math.radians(start), math.radians(stop)
    x0, y0 = x + r * math.cos(a0), y + r * math.sin(a0)
    x1, y1 = x + r * math.cos(a1), y + r * math.sin(a1)
    large = 1 if abs(stop - start) > 180 else 0
    sweep = 1 if stop > start else 0
    svg.add('<path d="M %.1f %.1f A %.1f %.1f 0 %d %d %.1f %.1f" fill="none" '
            'stroke="%s" stroke-width="1.4" marker-end="url(#arrow)"/>' %
            (x0, y0, r, r, large, sweep, x1, y1, color))


def arrow(svg, x0, y0, x1, y1, color=TEXT, width=1.4):
    svg.add('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" '
            'stroke-width="%.2f" marker-end="url(#arrow)"/>' %
            (x0, y0, x1, y1, color, width))


def ground(svg, x, y, w=44):
    """a fixed support under (x, y)"""
    svg.line(x - w / 2, y, x + w / 2, y, width=1.6)
    for k in range(6):
        sx = x - w / 2 + 4 + k * (w - 8) / 5
        svg.line(sx, y, sx - 7, y + 8, width=1)


def ring(x, y, r, count, start=0.0):
    return [(x + r * math.cos(start + 2 * math.pi * k / count),
             y + r * math.sin(start + 2 * math.pi * k / count))
            for k in range(count)]


def lug_outline(cx, cy, r, length, half):
    """a round end of radius r at (cx, cy) with a shank of half width half
    running to +x for length"""
    s = math.sqrt(r * r - half * half)
    return ('M %.1f %.1f A %.1f %.1f 0 1 0 %.1f %.1f L %.1f %.1f '
            'L %.1f %.1f Z' %
            (cx + s, cy - half, r, r, cx + s, cy + half, cx + length,
             cy + half, cx + length, cy - half))


def caption(svg, x, y, title, lines):
    svg.text(x, y, title, size=14, weight='bold')
    svg.text(x, y + 22, lines, size=12.5, color=TEXT)


def revolute(out):
    svg = Svg(760, 300)
    # a lug, end on: its bore tied to the frame on the joint's axis
    cx, cy = 120, 140
    svg.path(lug_outline(cx, cy, 72, 170, 30) +
             ' M %.1f %.1f m -36 0 a 36 36 0 1 0 72 0 a 36 36 0 1 0 -72 0'
             % (cx, cy), fill=BODY, rule='evenodd')
    nodes = ring(cx, cy, 36, 20)
    spider(svg, cx, cy, nodes, ORANGE)
    frame_marker(svg, cx, cy)
    arc_arrow(svg, cx, cy, 52, 200, 320)
    svg.text(cx, 255, ['the bore, end on: the joint axis', 'out of the page'],
             size=12, color=MUTED, anchor='middle')

    # a clevis, in section: the frame between two lugs takes both bores
    x0, y0 = 330, 70
    for top in (y0, y0 + 110):
        svg.rect(x0, top, 120, 50)
        svg.rect(x0 + 40, top, 40, 50, fill='#ffffff', stroke=EDGE)
    fx, fy = x0 + 60, y0 + 80
    svg.line(fx, y0 - 15, fx, y0 + 175, color=MUTED, dash='5 4')
    clevis_nodes = []
    for top in (y0, y0 + 110):
        for k in range(5):
            clevis_nodes.append((x0 + 40, top + 5 + k * 10))
            clevis_nodes.append((x0 + 80, top + 5 + k * 10))
    spider(svg, fx, fy, clevis_nodes, ORANGE)
    frame_marker(svg, fx, fy)
    svg.text(fx, 255, ['a clevis in section: a frame between', 'two lugs ties both bores'],
             size=12, color=MUTED, anchor='middle')

    caption(svg, 490, 70, 'Revolute joint', [
        'Tied: the nodes on the cylindrical faces',
        'coaxial with the joint axis, bores before',
        'pins, those of the smallest radius, all',
        'of them.',
        'The frame moves with their mean motion',
        '(RBE3). The joint passes three forces and',
        'two moments into the body; the rotation',
        'about its axis is free.'])
    svg.write(os.path.join(out, 'flex_bc_revolute.svg'))


def spherical(out):
    svg = Svg(760, 300)
    # a socket in section: a block with a hemispherical cavity open to -x
    cx, cy, r = 110, 140, 48
    svg.path('M %.1f %.1f L %.1f %.1f L %.1f %.1f L %.1f %.1f '
             'A %.1f %.1f 0 0 0 %.1f %.1f Z' %
             (cx, cy - 75, cx + 110, cy - 75, cx + 110, cy + 75, cx, cy + 75,
              r, r, cx, cy - r) +
             ' M %.1f %.1f L %.1f %.1f' % (cx, cy + 75, cx, cy + r),
             fill=BODY)
    svg.path('M %.1f %.1f A %.1f %.1f 0 0 1 %.1f %.1f' %
             (cx, cy - r, r, r, cx, cy + r), fill='#ffffff', stroke=EDGE)
    socket = [(cx + r * math.cos(a), cy + r * math.sin(a))
              for a in [math.radians(d) for d in range(-84, 85, 12)]]
    spider(svg, cx, cy, socket, RED)
    frame_marker(svg, cx, cy, 'root', dx=-48, dy=-12)
    svg.text(cx + 40, 255, ['a socket: the concave sphere', '(the link\'s root, held)'],
             size=12, color=MUTED, anchor='middle')

    # a ball in section on its shank
    bx, by, br = 360, 140, 44
    s = math.sqrt(br * br - 16 * 16)
    svg.path('M %.1f %.1f L %.1f %.1f A %.1f %.1f 0 1 1 %.1f %.1f '
             'L %.1f %.1f Z' %
             (bx - 130, by - 16, bx - s, by - 16, br, br, bx - s, by + 16,
              bx - 130, by + 16), fill=BODY)
    ball = ring(bx, by, br, 22)
    ball = [p for p in ball if p[0] > bx - s + 1]
    spider(svg, bx, by, ball, ORANGE)
    frame_marker(svg, bx, by, 'tip', dx=-8, dy=-54)
    svg.text(bx - 30, 255, ['a ball: the convex sphere', '(the link\'s tip)'],
             size=12, color=MUTED, anchor='middle')

    caption(svg, 490, 70, 'Spherical joint', [
        'Tied: the nodes on the spherical faces',
        'centred on the frame, a socket before a',
        'ball, those of the smallest radius.',
        'The joint passes three forces; the body',
        'turns freely about the centre. The',
        'attachment still has six static modes;',
        'the joint loads only its translations.'])
    svg.write(os.path.join(out, 'flex_bc_spherical.svg'))


def patch(out):
    svg = Svg(760, 300)
    # a frame inside the material: its node and the elements around it
    cx, cy = 120, 135
    svg.rect(30, 60, 190, 150)
    ring_nodes = ring(cx, cy, 42, 7, 0.3)
    outer = ring(cx, cy, 80, 9, 0.1)
    for a, b in zip(ring_nodes, ring_nodes[1:] + ring_nodes[:1]):
        svg.line(cx, cy, a[0], a[1], color=MUTED, width=0.8)
        svg.line(a[0], a[1], b[0], b[1], color=MUTED, width=0.8)
    for p in ring_nodes:
        near = sorted(outer, key=lambda q: (q[0] - p[0]) ** 2 +
                      (q[1] - p[1]) ** 2)[:2]
        for q in near:
            if 30 < q[0] < 220 and 60 < q[1] < 210:
                svg.line(p[0], p[1], q[0], q[1], color='#aab0b8', width=0.7)
    mids = [((cx + a[0]) / 2, (cy + a[1]) / 2) for a in ring_nodes]
    spider(svg, cx, cy, ring_nodes + mids, GREEN)
    frame_marker(svg, cx, cy, 'middle', dx=10, dy=-12)
    svg.text(cx + 5, 255, ['inside the material: the frame is a',
                           'node, tied to the elements around it'],
             size=12, color=MUTED, anchor='middle')

    # a frame outside the material: the nodes within radius
    ex, ey = 345, 70
    svg.rect(ex - 70, ey + 70, 160, 90)
    fx, fy = ex + 10, ey + 40
    svg.circle(fx, fy, 62, stroke=MUTED, dash='5 4')
    inside = [(ex - 40 + 18 * k, ey + 70) for k in range(6)]
    inside = [p for p in inside if (p[0] - fx) ** 2 + (p[1] - fy) ** 2 < 62 ** 2]
    inside += [(ex - 22 + 18 * k, ey + 88) for k in range(4)]
    inside = [p for p in inside if (p[0] - fx) ** 2 + (p[1] - fy) ** 2 < 62 ** 2]
    spider(svg, fx, fy, inside, BLUE)
    frame_marker(svg, fx, fy, 'sensor', dx=10, dy=-10)
    svg.line(fx, fy, fx + 44, fy - 44, color=MUTED, width=1)
    svg.text(fx + 34, fy - 48, 'radius', size=11, color=MUTED)
    svg.text(ex + 10, 255, ['outside it: the nodes within radius',
                            '(element_size by default)'],
             size=12, color=MUTED, anchor='middle')

    caption(svg, 490, 70, 'Float, rigid, chords, other frames', [
        'No bearing surface: a frame inside the',
        'material is made a node of the mesh and',
        'tied to the nodes of the elements that',
        'meet there; any other frame to the nodes',
        'within radius. So is a revolute or',
        'spherical frame with no matching face.',
        'A float joint passes nothing, but its',
        'frame moves with the deformed body.'])
    svg.write(os.path.join(out, 'flex_bc_patch.svg'))


def prismatic(out):
    svg = Svg(760, 330)
    x0, x1 = 40, 470
    top, bottom, wall = 70, 190, 26
    svg.rect(x0, top - wall, x1 - x0, wall)
    svg.rect(x0, bottom, x1 - x0, wall)
    axis_y = (top + bottom) / 2
    svg.line(x0 - 15, axis_y, x1 + 15, axis_y, color=MUTED, dash='6 4')
    stations = 5
    colors = [ORANGE, BLUE, GREEN, PURPLE, '#f2d933']
    length = (x1 - x0) / stations
    points = []
    for k in range(stations):
        a = x0 + k * length
        mid = a + length / 2
        if k:
            svg.line(a, top - wall, a, top, color=MUTED, dash='3 3')
            svg.line(a, bottom, a, bottom + wall, color=MUTED, dash='3 3')
        nodes = [(a + 6 + j * (length - 12) / 4, y)
                 for j in range(5) for y in (top, bottom)]
        spider(svg, mid, axis_y, nodes, colors[k], opacity=0.5)
        svg.circle(mid, axis_y, 4.5, fill=colors[k], stroke=TEXT, width=1)
        points.append((mid, axis_y))
    # the spline through the stations, deflected
    bend = [0, 7, 11, 9, 3]
    d = 'M %.1f %.1f' % (x0, axis_y + bend[0] - 4)
    for (px, py), b in zip(points, bend):
        d += ' L %.1f %.1f' % (px, py + b)
    d += ' L %.1f %.1f' % (x1, axis_y + 1)
    svg.path(d, stroke=TEXT, width=1.4, dash='2 3')
    # the plunger: two lands, its bearing
    px = 300
    for a, b in ((px - 48, px - 16), (px + 16, px + 48)):
        svg.rect(a, top + 3, b - a, bottom - top - 6, fill='#9fb4cc')
    svg.rect(px - 48, axis_y - 14, 96, 28, fill='#9fb4cc')
    frame_marker(svg, px, axis_y, 'slide', dx=-20, dy=-22)
    arrow(svg, px + 60, axis_y + 34, px + 120, axis_y + 34)
    for a, b in ((px - 48, px - 16), (px + 16, px + 48)):
        svg.line(a, bottom + wall + 12, b, bottom + wall + 12, width=2.2,
                 color=TEXT)
    svg.text(px, bottom + wall + 30, 'bearing (the lands)', size=12,
             color=MUTED, anchor='middle')
    svg.text(x0, 300, ['the bore in section: its wall cut into stations, each tied to a point on the',
                       'axis; the stations\' motion is interpolated by a cubic spline (dotted)'],
             size=12, color=MUTED)

    caption(svg, 505, 60, 'Prismatic joint', [
        'The body holding the bore gets a',
        'sliding attachment: rays from the',
        'frame find the wall, which is cut into',
        'stations (RBE3, six static modes',
        'each). The child moves with the mean',
        'motion of the wall under its bearing,',
        'wherever it is along the bore.'])
    svg.write(os.path.join(out, 'flex_bc_prismatic.svg'))


def link(svg, x, y, length, bend=None, color=BODY):
    """a link from a bore at (x, y) to one at (x + length, y), its centre
    line bent by bend(t), t from 0 to 1"""
    bend = bend or (lambda t: 0.0)
    steps = 24
    upper, lower = [], []
    for i in range(steps + 1):
        t = i / steps
        cx = x + t * length
        cy = y + bend(t)
        upper.append((cx, cy - 10))
        lower.append((cx, cy + 10))
    d = 'M %.1f %.1f ' % upper[0] + ' '.join(
        'L %.1f %.1f' % p for p in upper[1:])
    d += ' ' + ' '.join('L %.1f %.1f' % p for p in reversed(lower)) + ' Z'
    svg.path(d, fill=color)
    for t in (0.0, 1.0):
        bx, by = x + t * length, y + bend(t)
        svg.circle(bx, by, 22, fill=color, stroke=EDGE, width=1.2)
        svg.circle(bx, by, 10, fill='#ffffff', stroke=EDGE, width=1.2)
    return (x, y + bend(0.0)), (x + length, y + bend(1.0))


def reduction(out):
    svg = Svg(780, 300)
    panels = [
        ('Held', ['the reference attachment', 'carries the body\'s frame']),
        ('Static mode', ['B moves by a unit', 'amount, A held']),
        ('Fixed-interface mode', ['A and B held,', 'the body vibrates']),
    ]
    for p, (title, lines) in enumerate(panels):
        x = 30 + p * 255
        y = 120
        if p == 0:
            (ax, ay), (bx, by) = link(svg, x + 25, y, 160)
        elif p == 1:
            (ax, ay), (bx, by) = link(
                svg, x + 25, y, 160,
                lambda t: 34 * (3 * t * t - 2 * t * t * t))
            link(svg, x + 25, y, 160, color='none')
        else:
            (ax, ay), (bx, by) = link(
                svg, x + 25, y, 160, lambda t: 22 * math.sin(math.pi * t))
            link(svg, x + 25, y, 160, color='none')
        spider(svg, ax, ay, ring(ax, ay, 10, 10), RED, dots=True)
        frame_marker(svg, ax, ay)
        ground(svg, ax, ay + 32)
        svg.line(ax, ay + 22, ax, ay + 32, width=1.6)
        svg.text(ax - 6, ay - 30, 'A', size=13, weight='bold')
        spider(svg, bx, by, ring(bx, by, 10, 10), ORANGE, dots=True)
        frame_marker(svg, bx, by)
        if p == 1:
            svg.text(bx + 26, by + 5, 'B', size=13, weight='bold')
        else:
            svg.text(bx - 4, by - 30, 'B', size=13, weight='bold')
        if p == 1:
            arrow(svg, bx - 34, y, bx - 34, by - 4)
            svg.text(bx - 48, (y + by) / 2 + 4, '1', size=13)
        if p == 2:
            ground(svg, bx, by + 32)
            svg.line(bx, by + 22, bx, by + 32, width=1.6)
        svg.text(x + 105, 215, title, size=14, weight='bold', anchor='middle')
        svg.text(x + 105, 237, lines, size=12.5, color=MUTED, anchor='middle')
    svg.write(os.path.join(out, 'flex_bc_reduction.svg'))


def spiders(out):
    svg = Svg(620, 400)
    for p, (title, lines, rigid) in enumerate([
            ('Averaging (RBE3), the default',
             ['the frame moves with the mean motion of',
              'the nodes; no stiffness is added, so the',
              'bore can ovalize under the pin\'s load'], False),
            ('Rigid (RBE2), "spider": "rbe2"',
             ['the nodes move as one rigid body with',
              'the frame, as a press-fitted bush would;',
              'the bore stays round, stiffening the part'], True)]):
        cx, cy = 110, 110 + p * 190
        svg.circle(cx, cy, 70, fill=BODY, stroke=EDGE, width=1.2)
        if rigid:
            bore = ring(cx, cy, 36, 20)
            svg.circle(cx, cy, 36, fill='#ffffff', stroke=EDGE, width=1.2)
        else:
            bore = [(cx + 41 * math.cos(2 * math.pi * k / 20),
                     cy + 31 * math.sin(2 * math.pi * k / 20))
                    for k in range(20)]
            svg.add('<ellipse cx="%.1f" cy="%.1f" rx="41" ry="31" '
                    'fill="#ffffff" stroke="%s" stroke-width="1.2"/>' %
                    (cx, cy, EDGE))
            svg.circle(cx, cy, 36, stroke=MUTED, dash='4 3')
        spider(svg, cx, cy, bore, ORANGE, width=2.4 if rigid else 0.9,
               opacity=0.9 if rigid else 0.6)
        frame_marker(svg, cx, cy)
        arrow(svg, cx, cy - 92, cx, cy - 74)
        svg.text(cx + 6, cy - 82, 'load', size=11, color=MUTED)
        svg.text(cx + 105, cy - 40, title, size=14, weight='bold')
        svg.text(cx + 105, cy - 16, lines, size=12.5)
    svg.write(os.path.join(out, 'flex_bc_spiders.svg'))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'doc', 'sphinx',
        'images', 'flex')
    os.makedirs(out, exist_ok=True)
    reduction(out)
    spiders(out)
    revolute(out)
    spherical(out)
    patch(out)
    prismatic(out)


if __name__ == '__main__':
    main()
