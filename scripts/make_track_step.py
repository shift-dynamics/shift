#!/usr/bin/env python3
"""Write the STEP files of an excavator track for the tests of contact on
STEP surfaces: examples/cad/sprocket.step (the drive sprocket),
examples/cad/idler.step (the front idler) and examples/cad/track_link.step
(one link of the track: link plates, bushing, pin and a triple grouser shoe),
and the models of a whole track loop on them, examples/cad/track_loop.json,
track_frame.json and, on soil, track_frame_soil.json.

The sizes are those of a 20 t excavator: a track pitch of 190 mm, a bushing
of 59 mm diameter and a shoe 600 mm wide. The sprocket has 21 teeth and
engages every second one (a hunting tooth sprocket, so each tooth is used
half as often), so its pitch circle passes through the bushings' centers
190 mm apart: radius 95 / sin(2 pi / 21) = 322.3 mm.

Every solid is a profile of lines and circular arcs in a plane, extruded
along the plane's normal: planar faces at its ends, and a planar face for
each line and a cylindrical face for each arc around its sides. Full circles
are two half arcs, so no face needs a seam. The files hold the 3D geometry
only; shift computes the curves of the edges in the faces' parameters
(cad::Repair).

The track loop has 44 links around the sprocket and an idler of the same
pitch radius, on the ground, the sprocket driven at a constant speed by a
PID controller. The model follows shift's convention, x forward, y to the
right and z down: the ground the plane z = 0, the wheels' axles along y
above it, the idler in front of the sprocket (towards +x). The initial
positions of the links are those of a closed
loop; shift's loop closure removes the rounding.

Usage: make_track_step.py [output directory] (default examples/cad)
"""

import json
import math
import os
import sys

# ---------------------------------------------------------------------------
# dimensions, mm

PITCH = 190.0               # track pitch: bushing to pin
BUSHING_RADIUS = 29.5
BUSHING_BORE = 20.0
PIN_RADIUS = 19.0
BUSHING_HALF_LENGTH = 60.0  # between the inner link plates
PLATE_THICKNESS = 35.0
PLATE_HALF_HEIGHT = 50.0
SHOE_HALF_WIDTH = 300.0
SHOE_GAP = 5.0              # between neighboring shoes, at each end
SHOE_TOP = 50.0             # the shoe is bolted under the plates
SHOE_THICKNESS = 16.0
GROUSER_HEIGHT = 26.0
GROUSER_THICKNESS = 20.0

TEETH = 21
SEAT_RADIUS = 30.5          # the bushing's seat, with 1 mm of clearance
SEAT_HALF_ANGLE = math.radians(68.0)  # of the seat arc, from its bottom
FLANK_RADIUS = 200.0
TIP_HEIGHT = 30.0           # of the tip circle above the pitch circle
SPROCKET_HALF_WIDTH = 30.0
BORE_RADIUS = 100.0
IDLER_CLEARANCE = 1.0       # of the bushings above the idler's tread

# rollers: two treads on the link plates (the rails, 60 to 130 mm either
# side of the middle), between flanges outside them, and a recessed hub
ROLLER_TREAD_INNER = 56.0
ROLLER_TREAD_OUTER = 134.0
ROLLER_FLANGE_OUTER = 154.0
TRACK_ROLLER_RADIUS = 100.0
CARRIER_ROLLER_RADIUS = 70.0
FLANGE_HEIGHT = 25.0
HUB_RECESS = 10.0
RAIL_GAP = 0.5              # of the rollers above the rails at the start
TRACK_ROLLERS = 5
FRAME_MASS = 10000.0        # carried by one track: half of a 20 t machine

# the track loop
LOOP_SPAN_LINKS = 16        # links between the wheels, top and bottom
LOOP_GROUND_GAP = 0.5       # below the grousers of the bottom links
LOOP_SPEED = 1.0            # target of the sprocket's drive, rad/s
LOOP_E = 2e9                # Young's modulus of the contacts, a hundredth of
                            # steel's: the impacts of the bushings in the
                            # seats need steps 10 times smaller with steel's
LOOP_TIME = 2.0             # s

# the same track on soil, with contact on primitive shapes
SOIL_PIN_RADIUS = 16.0      # of the sprocket's and idler's pins, between seats
SOIL_RAIL_OFFSET = 170.0    # of each rail and roller tread from the middle:
                            # clear of the spheres of the wheels' treads
SOIL_RAIL_HALF_WIDTH = 30.0
SOIL_CONTACT_STIFFNESS = 1e9  # of the boxes under the rollers, N/m^1.5
SOIL_DRIVE_KP = 40000.0     # of the sprocket's drive
SOIL_DRIVE_KI = 80000.0
SOIL_TIME = 3.0             # s


# ---------------------------------------------------------------------------
# 2D geometry

def rotate(p, a):
    c, s = math.cos(a), math.sin(a)
    return (c * p[0] - s * p[1], s * p[0] + c * p[1])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def scale(a, s):
    return (a[0] * s, a[1] * s)


def norm(a):
    return math.hypot(a[0], a[1])


def angle(a):
    return math.atan2(a[1], a[0])


def circle_intersections(c0, r0, c1, r1):
    d = norm(sub(c1, c0))
    a = (r0 * r0 - r1 * r1 + d * d) / (2.0 * d)
    h = math.sqrt(r0 * r0 - a * a)
    m = add(c0, scale(sub(c1, c0), a / d))
    off = scale((-(c1[1] - c0[1]) / d, (c1[0] - c0[0]) / d), h)
    return [add(m, off), sub(m, off)]


def ccw_from(a0, a):
    """angle from a0 to a, counterclockwise, in [0, 2 pi)"""
    return (a - a0) % (2.0 * math.pi)


# A profile is a closed loop of segments, each ('line', start) or
# ('arc', start, center, ccw), running from its start to the next one's.
# Outer loops run counterclockwise, holes clockwise.

def circle_loop(center, radius, ccw):
    a = add(center, (radius, 0.0))
    b = add(center, (-radius, 0.0))
    if ccw:
        return [('arc', a, center, True), ('arc', b, center, True)]
    return [('arc', a, center, False), ('arc', b, center, False)]


def polygon_loop(points):
    return [('line', p) for p in points]


def sprocket_profile():
    """the outline of the sprocket's teeth, counterclockwise: for each seat
    its leading flank, the seat, its trailing flank and the tip that follows"""
    pitch_radius = (PITCH / 2.0) / math.sin(2.0 * math.pi / TEETH)
    tip_radius = pitch_radius + TIP_HEIGHT
    seat = (pitch_radius, 0.0)
    # the seat arc, clockwise about the seat's center through its bottom
    # (towards the sprocket's center), from -half angle to +half angle
    toward_next = (math.cos(math.pi - SEAT_HALF_ANGLE),
                   math.sin(math.pi - SEAT_HALF_ANGLE))
    seat_end = add(seat, scale(toward_next, SEAT_RADIUS))
    # the trailing flank: an arc touching the seat arc from outside, its
    # center beyond the seat's end, to the tip circle
    flank_center = add(seat, scale(toward_next, SEAT_RADIUS + FLANK_RADIUS))
    start = angle(sub(seat_end, flank_center))
    candidates = circle_intersections(flank_center, FLANK_RADIUS, (0.0, 0.0),
                                      tip_radius)
    tip_start = min(candidates,
                    key=lambda p: ccw_from(start, angle(sub(p, flank_center))))
    tooth_center = math.pi / TEETH
    if not angle(tip_start) < tooth_center:
        sys.exit('the flanks meet below the tip circle')

    def mirror(p):  # about the seat's radial line, the x axis
        return (p[0], -p[1])

    segments = []
    for k in range(TEETH):
        a = 2.0 * math.pi * k / TEETH
        r = lambda p: rotate(p, a)
        # leading flank, from the tip down to the seat (counterclockwise
        # about its center, the mirror image of the trailing one)
        segments.append(('arc', r(mirror(tip_start)), r(mirror(flank_center)),
                         True))
        segments.append(('arc', r(mirror(seat_end)), r(seat), False))
        segments.append(('arc', r(seat_end), r(flank_center), True))
        segments.append(('arc', r(tip_start), (0.0, 0.0), True))
    return segments, pitch_radius, tip_radius


def shoe_profile():
    """the shoe's cross section in (x along the track, z out of the track
    loop): a plate with three grousers, counterclockwise in (x, z)"""
    x0, x1 = SHOE_GAP, PITCH - SHOE_GAP
    z0, z1 = SHOE_TOP, SHOE_TOP + SHOE_THICKNESS
    zg = z1 + GROUSER_HEIGHT
    h = GROUSER_THICKNESS / 2.0
    centers = [x0 + h + 5.0, PITCH / 2.0, x1 - h - 5.0]
    points = [(x0, z0), (x1, z0), (x1, z1)]
    for c in reversed(centers):
        points += [(c + h, z1), (c + h, zg), (c - h, zg), (c - h, z1)]
    points.append((x0, z1))
    return polygon_loop(dedupe(points))


def dedupe(points):
    out = []
    for p in points:
        if not out or norm(sub(p, out[-1])) > 1e-9:
            out.append(p)
    if norm(sub(out[0], out[-1])) < 1e-9:
        out.pop()
    return out


def plate_profile(round_end, square_end):
    """a link plate in (x, z): a rectangle from square_end to round_end with
    a half circle about (round_end, 0), counterclockwise"""
    h = PLATE_HALF_HEIGHT
    c = (round_end, 0.0)
    if round_end < square_end:  # the half circle on the left
        return [('line', (square_end, -h)), ('line', (square_end, h)),
                ('arc', (round_end, h), c, True), ('line', (round_end, -h))]
    return [('line', (square_end, h)), ('line', (square_end, -h)),
            ('arc', (round_end, -h), c, True), ('line', (round_end, h))]


# ---------------------------------------------------------------------------
# STEP

class Step:
    def __init__(self):
        self.lines = []
        self.next_id = 100
        self.points = {}
        self.directions = {}

    def add(self, text):
        i = self.next_id
        self.next_id += 1
        self.lines.append('#%d=%s;' % (i, text))
        return i

    @staticmethod
    def num(x):
        if abs(x) < 5e-13:
            x = 0.0
        s = repr(round(x, 9))
        if 'e' in s or 'E' in s:
            return '%.9E' % x
        if '.' not in s:
            s += '.'
        return s

    def point(self, p):
        key = tuple(round(c, 9) for c in p)
        if key not in self.points:
            self.points[key] = self.add("CARTESIAN_POINT('',(%s))" %
                                        ','.join(self.num(c) for c in p))
        return self.points[key]

    def direction(self, d):
        n = math.sqrt(sum(c * c for c in d))
        d = tuple(c / n for c in d)
        key = tuple(round(c, 12) for c in d)
        if key not in self.directions:
            self.directions[key] = self.add(
                "DIRECTION('',(%s))" % ','.join(self.num(c) for c in d))
        return self.directions[key]

    def placement(self, origin, z, x):
        return self.add("AXIS2_PLACEMENT_3D('',#%d,#%d,#%d)" %
                        (self.point(origin), self.direction(z),
                         self.direction(x)))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def extrude(step, name, loops, origin, e1, e2, length):
    """a solid: the profile loops (the outer one first) in the plane through
    origin spanned by e1 and e2, extruded by length along e3 = e1 x e2.
    Returns the MANIFOLD_SOLID_BREP."""
    e3 = cross(e1, e2)

    def to3(p, h):
        return tuple(origin[i] + p[0] * e1[i] + p[1] * e2[i] + h * e3[i]
                     for i in range(3))

    def dir3(d):
        return tuple(d[0] * e1[i] + d[1] * e2[i] for i in range(3))

    faces = []
    bottom_bounds, top_bounds = [], []
    for li, loop in enumerate(loops):
        n = len(loop)
        vb = [step.add("VERTEX_POINT('',#%d)" % step.point(to3(s[1], 0.0)))
              for s in loop]
        vt = [step.add("VERTEX_POINT('',#%d)" % step.point(to3(s[1], length)))
              for s in loop]
        # the edges along e3 at each vertex, from bottom to top
        up = []
        for i, s in enumerate(loop):
            line = step.add("LINE('',#%d,#%d)" % (
                step.point(to3(s[1], 0.0)),
                step.add("VECTOR('',#%d,%s)" % (step.direction(e3),
                                               step.num(length)))))
            up.append(step.add("EDGE_CURVE('',#%d,#%d,#%d,.T.)" %
                               (vb[i], vt[i], line)))
        bottom, top = [], []
        for i, s in enumerate(loop):
            j = (i + 1) % n
            a, b = s[1], loop[j][1]
            edges = []
            for h, v in ((0.0, vb), (length, vt)):
                if s[0] == 'line':
                    d = sub(b, a)
                    curve = step.add("LINE('',#%d,#%d)" % (
                        step.point(to3(a, h)),
                        step.add("VECTOR('',#%d,%s)" % (
                            step.direction(dir3(d)), step.num(norm(d))))))
                    sense = '.T.'
                else:
                    c, ccw = s[2], s[3]
                    r = norm(sub(a, c))
                    curve = step.add("CIRCLE('',#%d,%s)" % (
                        step.placement(to3(c, h), e3, dir3((1.0, 0.0))),
                        step.num(r)))
                    sense = '.T.' if ccw else '.F.'
                edges.append(step.add("EDGE_CURVE('',#%d,#%d,#%d,%s)" %
                                      (v[i], v[j], curve, sense)))
            bottom.append(edges[0])
            top.append(edges[1])
            # the side face: outward normal t x e3 for a line; for an arc
            # the cylinder about its center, its normal outward if the arc
            # runs counterclockwise (the solid on the center's side)
            if s[0] == 'line':
                d = sub(b, a)
                t = dir3(d)
                normal = cross(t, e3)
                surface = step.add("PLANE('',#%d)" % step.placement(
                    to3(a, 0.0), normal, t))
                same = '.T.'
            else:
                c, ccw = s[2], s[3]
                r = norm(sub(a, c))
                surface = step.add("CYLINDRICAL_SURFACE('',#%d,%s)" % (
                    step.placement(to3(c, 0.0), e3, dir3((1.0, 0.0))),
                    step.num(r)))
                same = '.T.' if ccw else '.F.'
            # counterclockwise about the outward normal: along the bottom,
            # up, back along the top, down
            oriented = [
                step.add("ORIENTED_EDGE('',*,*,#%d,.T.)" % edges[0]),
                step.add("ORIENTED_EDGE('',*,*,#%d,.T.)" % up[j]),
                step.add("ORIENTED_EDGE('',*,*,#%d,.F.)" % edges[1]),
                step.add("ORIENTED_EDGE('',*,*,#%d,.F.)" % up[i])]
            edge_loop = step.add("EDGE_LOOP('',(%s))" %
                                 ','.join('#%d' % e for e in oriented))
            bound = step.add("FACE_OUTER_BOUND('',#%d,.T.)" % edge_loop)
            faces.append(step.add("ADVANCED_FACE('',(#%d),#%d,%s)" %
                                  (bound, surface, same)))
        kind = 'FACE_OUTER_BOUND' if li == 0 else 'FACE_BOUND'
        # the top face's normal is e3: the loop as given; the bottom's is -e3:
        # reversed
        top_loop = step.add("EDGE_LOOP('',(%s))" % ','.join(
            '#%d' % step.add("ORIENTED_EDGE('',*,*,#%d,.T.)" % e)
            for e in top))
        bottom_loop = step.add("EDGE_LOOP('',(%s))" % ','.join(
            '#%d' % step.add("ORIENTED_EDGE('',*,*,#%d,.F.)" % e)
            for e in reversed(bottom)))
        top_bounds.append(step.add("%s('',#%d,.T.)" % (kind, top_loop)))
        bottom_bounds.append(step.add("%s('',#%d,.T.)" % (kind, bottom_loop)))
    first = loops[0][0][1]
    bottom_plane = step.add("PLANE('',#%d)" % step.placement(
        to3(first, 0.0), tuple(-c for c in e3), e1))
    top_plane = step.add("PLANE('',#%d)" % step.placement(
        to3(first, length), e3, e1))
    faces.append(step.add("ADVANCED_FACE('bottom',(%s),#%d,.T.)" % (
        ','.join('#%d' % b for b in bottom_bounds), bottom_plane)))
    faces.append(step.add("ADVANCED_FACE('top',(%s),#%d,.T.)" % (
        ','.join('#%d' % b for b in top_bounds), top_plane)))
    shell = step.add("CLOSED_SHELL('',(%s))" %
                     ','.join('#%d' % f for f in faces))
    return step.add("MANIFOLD_SOLID_BREP('%s',#%d)" % (name, shell))


def color(step, item, rgb):
    c = step.add("COLOUR_RGB('',%s)" % ','.join(step.num(x) for x in rgb))
    c = step.add("FILL_AREA_STYLE_COLOUR('',#%d)" % c)
    c = step.add("FILL_AREA_STYLE('',(#%d))" % c)
    c = step.add("SURFACE_STYLE_FILL_AREA(#%d)" % c)
    c = step.add("SURFACE_SIDE_STYLE('',(#%d))" % c)
    c = step.add("SURFACE_STYLE_USAGE(.BOTH.,#%d)" % c)
    c = step.add("PRESENTATION_STYLE_ASSIGNMENT((#%d))" % c)
    return step.add("STYLED_ITEM('color',(#%d),#%d)" % (c, item))


def write(path, product, comment, solids):
    """solids: (name, builder(step) -> MANIFOLD_SOLID_BREP, rgb)"""
    step = Step()
    items, styles = [], []
    for name, build, rgb in solids:
        solid = build(step, name)
        items.append(solid)
        styles.append(color(step, solid, rgb))
    origin = step.placement((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
    header = [
        'ISO-10303-21;',
        'HEADER;',
        '/* %s */' % comment,
        "FILE_DESCRIPTION(('%s'),'2;1');" % product,
        "FILE_NAME('%s','2026-11-04T00:00:00',('shift'),(''),"
        "'scripts/make_track_step.py','','');" % os.path.basename(path),
        "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));",
        'ENDSEC;',
        'DATA;',
        "#1=APPLICATION_CONTEXT('core data for automotive mechanical design "
        "processes');",
        "#2=APPLICATION_PROTOCOL_DEFINITION('international standard',"
        "'automotive_design',2000,#1);",
        "#3=PRODUCT_CONTEXT('',#1,'mechanical');",
        "#4=PRODUCT_DEFINITION_CONTEXT('part definition',#1,'design');",
        "#5=PRODUCT('%s','%s','',(#3));" % (product, product),
        "#6=PRODUCT_DEFINITION_FORMATION('','',#5);",
        "#7=PRODUCT_DEFINITION('design','',#6,#4);",
        "#8=PRODUCT_DEFINITION_SHAPE('','',#7);",
        '#9=SHAPE_DEFINITION_REPRESENTATION(#8,#20);',
        '#10=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));',
        '#11=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));',
        '#12=(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT());',
        "#13=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-06),#10,"
        "'distance_accuracy_value','');",
        '#14=(GEOMETRIC_REPRESENTATION_CONTEXT(3)'
        'GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#13))'
        'GLOBAL_UNIT_ASSIGNED_CONTEXT((#10,#11,#12))'
        "REPRESENTATION_CONTEXT('',''));",
        "#20=ADVANCED_BREP_SHAPE_REPRESENTATION('%s',(%s),#14);" % (
            product, ','.join('#%d' % i for i in items + [origin])),
        "#21=MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',"
        "(%s),#14);" % ','.join('#%d' % s for s in styles),
    ]
    with open(path, 'w') as f:
        f.write('\n'.join(header + step.lines +
                          ['ENDSEC;', 'END-ISO-10303-21;']) + '\n')


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'examples', 'cad')

    teeth, pitch_radius, tip_radius = sprocket_profile()
    bore = circle_loop((0.0, 0.0), BORE_RADIUS, False)
    write(os.path.join(out, 'sprocket.step'), 'sprocket',
          ('Drive sprocket of an excavator track (20 t class): %d teeth, '
           'engaging the bushings of a %.0f mm pitch track at every second '
           'tooth; pitch circle radius %.1f mm, tip circle %.1f mm, bore '
           '%.0f mm, %.0f mm wide. Axis along z through the origin, the '
           'middle of its width at z = 0, a seat on the x axis. Each tooth: '
           'two flanks (arcs of radius %.0f mm) and a tip (an arc of the tip '
           'circle) between seats (arcs of radius %.1f mm about the pitch '
           'circle). In millimeters. Written by scripts/make_track_step.py '
           'for the tests of contact on STEP surfaces.' %
           (TEETH, PITCH, pitch_radius, tip_radius, 2 * BORE_RADIUS,
            2 * SPROCKET_HALF_WIDTH, FLANK_RADIUS, SEAT_RADIUS)),
          [('sprocket',
            lambda s, n: extrude(s, n, [teeth, bore],
                                 (0.0, 0.0, -SPROCKET_HALF_WIDTH),
                                 (1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                                 2 * SPROCKET_HALF_WIDTH),
            (0.45, 0.47, 0.5))])

    # the link, in (x along the track, y along the pin, z out of the track
    # loop): its bushing about the y axis, its pin at x = PITCH. Profiles in
    # (x, z) are extruded along x cross z = -y, from y = top.
    def xz(name, loops, y_top, width, rgb):
        return (name,
                lambda s, n: extrude(s, n, loops, (0.0, y_top, 0.0),
                                     (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), width),
                rgb)

    inner = plate_profile(0.0, PITCH / 2.0)
    outer = plate_profile(PITCH, PITCH / 2.0)
    inner_bore = circle_loop((0.0, 0.0), BUSHING_BORE, False)
    outer_bore = circle_loop((PITCH, 0.0), PIN_RADIUS, False)
    t = PLATE_THICKNESS
    b = BUSHING_HALF_LENGTH
    link_color = (0.25, 0.25, 0.27)
    solids = [
        xz('bushing', [circle_loop((0.0, 0.0), BUSHING_RADIUS, True),
                       circle_loop((0.0, 0.0), BUSHING_BORE, False)],
           b, 2 * b, (0.7, 0.7, 0.72)),
        xz('pin', [circle_loop((PITCH, 0.0), PIN_RADIUS, True)],
           b + 2 * t, 2 * (b + 2 * t), (0.8, 0.8, 0.82)),
        # the inner plates have the bushing's bore, through which the next
        # link's pin passes; the outer plates hold the pin
        xz('inner plate left', [inner, inner_bore], b + t, t, link_color),
        xz('inner plate right', [inner, inner_bore], -b, t, link_color),
        xz('outer plate left', [outer, outer_bore], b + 2 * t, t, link_color),
        xz('outer plate right', [outer, outer_bore], -b - t, t, link_color),
        xz('shoe', [shoe_profile()], SHOE_HALF_WIDTH, 2 * SHOE_HALF_WIDTH,
           (0.15, 0.15, 0.15)),
    ]
    write(os.path.join(out, 'track_link.step'), 'track_link',
          ('One link of an excavator track (20 t class), %.0f mm pitch: a '
           'bushing (%.0f mm diameter, %.0f mm long) about the y axis between '
           'the inner link plates, a pin (%.0f mm diameter) at x = %.0f mm '
           'between the outer plates, which straddle the next link\'s inner '
           'plates, and a triple grouser shoe %.0f mm wide (%.0f mm plate, '
           '%.0f mm grousers) at z = %.0f mm, z pointing out of the track '
           'loop. In millimeters. Written by scripts/make_track_step.py for '
           'the tests of contact on STEP surfaces.' %
           (PITCH, 2 * BUSHING_RADIUS, 2 * b, 2 * PIN_RADIUS, PITCH,
            2 * SHOE_HALF_WIDTH, SHOE_THICKNESS, GROUSER_HEIGHT, SHOE_TOP)),
          solids)

    tread = pitch_radius - BUSHING_RADIUS - IDLER_CLEARANCE
    write(os.path.join(out, 'idler.step'), 'idler',
          ('Front idler of an excavator track (20 t class): a plain wheel '
           'of %.1f mm tread radius, under the bushings of a %.0f mm pitch '
           'track with %.0f mm of clearance on the pitch circle of the '
           'sprocket (%.1f mm), bore %.0f mm, %.0f mm wide. Axis along z '
           'through the origin, the middle of its width at z = 0. In '
           'millimeters. Written by scripts/make_track_step.py for the tests '
           'of contact on STEP surfaces.' %
           (tread, PITCH, IDLER_CLEARANCE, pitch_radius, 2 * BORE_RADIUS,
            2 * SPROCKET_HALF_WIDTH)),
          [('idler',
            lambda s, n: extrude(s, n, [circle_loop((0.0, 0.0), tread, True),
                                        circle_loop((0.0, 0.0), BORE_RADIUS,
                                                    False)],
                                 (0.0, 0.0, -SPROCKET_HALF_WIDTH),
                                 (1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                                 2 * SPROCKET_HALF_WIDTH),
            (0.45, 0.47, 0.5))])

    track_loop(os.path.join(out, 'track_loop.json'), pitch_radius)

    for name, radius in (('track_roller', TRACK_ROLLER_RADIUS),
                         ('carrier_roller', CARRIER_ROLLER_RADIUS)):
        write(os.path.join(out, name + '.step'), name,
              ('%s of an excavator track (20 t class), double flanged: two '
               'treads of %.0f mm radius from %.0f to %.0f mm either side of '
               'its middle, on the link plates (the rails), flanges %.0f mm '
               'high outside them to %.0f mm, and a hub recessed by %.0f mm '
               'between the treads. Axis along z through the origin, the '
               'middle at z = 0. In millimeters. Written by '
               'scripts/make_track_step.py for the tests of contact on STEP '
               'surfaces.' %
               (name.replace('_', ' ').capitalize(), radius,
                ROLLER_TREAD_INNER, ROLLER_TREAD_OUTER, FLANGE_HEIGHT,
                ROLLER_FLANGE_OUTER, HUB_RECESS)),
              roller_solids(radius))

    track_loop(os.path.join(out, 'track_frame.json'), pitch_radius,
               frame=True)
    track_loop(os.path.join(out, 'track_frame_soil.json'), pitch_radius,
               frame=True, soil=True)


# the rotation from the sprocket's frame (X up, Y along the track to the
# rear, Z its axis) to the model's (x forward, y to the right, z down: shift's
# convention): rows
WHEEL_FRAME = [[0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]]


def to_model(v):
    return [sum(WHEEL_FRAME[i][k] * v[k] for k in range(3)) for i in range(3)]


def roller_solids(radius):
    """a roller's discs along z: flange, tread, hub, tread, flange"""
    def disc(name, r, z0, z1, rgb):
        return (name,
                lambda s, n: extrude(s, n, [circle_loop((0.0, 0.0), r, True)],
                                     (0.0, 0.0, z0), (1.0, 0.0, 0.0),
                                     (0.0, 1.0, 0.0), z1 - z0),
                rgb)
    steel = (0.55, 0.56, 0.58)
    flange = radius + FLANGE_HEIGHT
    a, b, c = ROLLER_TREAD_INNER, ROLLER_TREAD_OUTER, ROLLER_FLANGE_OUTER
    return [disc('flange left', flange, b, c, steel),
            disc('tread left', radius, a, b, steel),
            disc('hub', radius - HUB_RECESS, -a, a, steel),
            disc('tread right', radius, -b, -a, steel),
            disc('flange right', flange, -c, -b, steel)]


def roller_mass(radius):
    """mass, radial and axial moments of inertia of a roller of steel"""
    parts = []
    flange = radius + FLANGE_HEIGHT
    for r, z0, z1 in ((flange, ROLLER_TREAD_OUTER, ROLLER_FLANGE_OUTER),
                      (radius, ROLLER_TREAD_INNER, ROLLER_TREAD_OUTER),
                      (radius - HUB_RECESS, -ROLLER_TREAD_INNER,
                       ROLLER_TREAD_INNER),
                      (radius, -ROLLER_TREAD_OUTER, -ROLLER_TREAD_INNER),
                      (flange, -ROLLER_FLANGE_OUTER, -ROLLER_TREAD_OUTER)):
        r, z0, z1 = r / 1000.0, z0 / 1000.0, z1 / 1000.0
        m = 7850.0 * math.pi * r * r * (z1 - z0)
        zc = 0.5 * (z0 + z1)
        radial = m * (3.0 * r * r + (z1 - z0) ** 2) / 12.0 + m * zc * zc
        parts.append((m, radial, 0.5 * m * r * r))
    return tuple(sum(x[i] for x in parts) for i in range(3))


def track_loop(path, pitch_radius, frame=False, soil=False):
    """the model of a track loop around the sprocket and an idler of the
    same pitch radius, in meters: x forward, z down, the axles along y.
    The loop is laid out in the plane of the wheels, in the sprocket's frame
    (X up, Y along the track to the rear, Z its axis), then transformed to the model's
    coordinates by WHEEL_FRAME. Everything starts at rest, the bushings 1 mm off the seats and
    the tread, the shoes LOOP_GROUND_GAP off the ground; the PID controller
    brings the sprocket up to speed. Six bushings sit on each wheel, 4 pi / 21 apart, the first and
    last pi / 42 past the top and bottom, and LOOP_SPAN_LINKS + 1 links
    span the straight between them, so the loop closes exactly.

    With frame, the wheels are on a track frame (the machine's half,
    FRAME_MASS) free to move in the plane of the track (forward, up and down,
    and pitching), with TRACK_ROLLERS track rollers on the rails of the
    bottom links and a carrier roller under the rails of the top links,
    RAIL_GAP off them; the contacts wear, and a track module logs the sag of
    the top spans, the misguide of the links and the side force on the
    rollers' flanges.

    With soil (and frame), the track runs on soil instead of the ground
    plane: a track_soil module under every shoe. The STEP files are only
    drawn. The contacts are on primitive shapes, which make it a fast
    example where the STEP surfaces make a benchmark: a sphere at each
    bushing, on a sphere as the tread of each wheel and between pins at its
    pitch radius, so that it sits in a seat; two boxes as each link's rails
    and two spheres as each roller's treads."""
    p = PITCH / 1000.0
    r = pitch_radius / 1000.0
    step = 4.0 * math.pi / TEETH
    first = math.pi / 42.0
    n = LOOP_SPAN_LINKS
    # the idler's center, its last bushing the span's length from the
    # sprocket's first
    y_idler = 2.0 * r * math.sin(first) - (n + 1) * p

    # the bushings around the loop: down the far side of the sprocket,
    # along the bottom, up around the idler and back along the top
    bushings = []
    for k in range(6):
        a = first + k * step
        bushings.append((r * math.cos(a), r * math.sin(a)))
    last = bushings[-1]
    for k in range(1, n + 1):
        bushings.append((last[0], last[1] - k * p))
    for k in range(6):
        a = math.pi + first + k * step
        bushings.append((r * math.cos(a), y_idler + r * math.sin(a)))
    last = bushings[-1]
    for k in range(1, n + 1):
        bushings.append((last[0], last[1] + k * p))
    links = len(bushings)

    def direction(i):
        a, b = bushings[i], bushings[(i + 1) % links]
        d = sub(b, a)
        return scale(d, 1.0 / norm(d))

    # gap of the loop, along the pitch: zero when it closes
    gap = norm(sub(bushings[0], bushings[-1])) - p
    if abs(gap) > 1e-12:
        raise RuntimeError('the track loop does not close: %g m' % gap)

    def number(x):
        return float('%.15g' % x)

    # the ground is the plane z = 0; the axles are this high above it, the
    # bottom links' grousers LOOP_GROUND_GAP off it
    axle_height = (r * math.cos(first) +
                   (SHOE_TOP + SHOE_THICKNESS + GROUSER_HEIGHT +
                    LOOP_GROUND_GAP) / 1000.0)

    def place(v):
        # from the sprocket's frame to the model's, z down
        w = to_model(v)
        return [w[0], w[1], w[2] - axle_height]

    def inertia(mass, ixx, iyy, izz):
        return {'mass': mass, 'Ixx': ixx, 'Iyy': iyy, 'Izz': izz,
                'Ixy': 0.0, 'Iyz': 0.0, 'Izx': 0.0}

    # steel wheels: the sprocket as in the tests; the idler a disc of the
    # tread radius with the bore
    tread = r - BUSHING_RADIUS / 1000.0 - IDLER_CLEARANCE / 1000.0
    bore = BORE_RADIUS / 1000.0
    width = 2.0 * SPROCKET_HALF_WIDTH / 1000.0
    idler_mass = 7850.0 * math.pi * (tread ** 2 - bore ** 2) * width
    idler_radial = idler_mass / 12.0 * (3.0 * (tread ** 2 + bore ** 2) +
                                        width ** 2)
    idler_axial = idler_mass / 2.0 * (tread ** 2 + bore ** 2)
    contact = [{'type': 'cad', 'material': 'steel'}]

    def sphere(radius, at=(0.0, 0.0, 0.0)):
        return {'type': 'sphere', 'radius': number(radius),
                'material': 'steel',
                'pose': {'T': [number(v) for v in at]}}

    def pins(offset):
        # a pin between each two seats of a wheel, seats at offset
        half = math.pi / TEETH
        return [sphere(tread)] + [
            sphere(SOIL_PIN_RADIUS / 1000.0,
                   (r * math.cos(offset + half * (2 * k + 1)),
                    r * math.sin(offset + half * (2 * k + 1)), 0.0))
            for k in range(TEETH)]

    bodies = [
        {'name': 'sprocket',
         'geometry': [{'type': 'cad', 'filename': 'sprocket.step'}],
         'collision_geometry': pins(0.0) if soil else contact,
         'center_of_mass': [0.0, 0.0, 0.0],
         'inertia': inertia(130.7, 3.8, 3.8, 7.5)},
        {'name': 'idler',
         'geometry': [{'type': 'cad', 'filename': 'idler.step'}],
         'collision_geometry': pins(math.pi + first) if soil else contact,
         'center_of_mass': [0.0, 0.0, 0.0],
         'inertia': inertia(number(idler_mass), number(idler_radial),
                            number(idler_radial), number(idler_axial))},
    ]
    if not soil:
        bodies.append(
            {'name': 'ground', 'geometry': [],
             'collision_geometry': [{
                 'name': 'ground', 'type': 'plane',
                 'normal': [0.0, 0.0, -1.0], 'material': 'steel'}],
             'center_of_mass': [0.0, 0.0, 0.0],
             'inertia': inertia(1.0, 1.0, 1.0, 1.0)})
    # rollers in the plane of the wheels (X, Y): track rollers between the
    # sprocket and the idler, their treads on the rails of the bottom links,
    # 50 mm inside the bushings; a carrier roller under the top links' rails
    rail = PLATE_HALF_HEIGHT / 1000.0
    gap = RAIL_GAP / 1000.0
    rt = TRACK_ROLLER_RADIUS / 1000.0
    rc = CARRIER_ROLLER_RADIUS / 1000.0
    rollers = []
    if frame:
        bottom = -r * math.cos(first) + rail + rt + gap
        y0, y1 = -0.5, y_idler + 0.45
        for k in range(TRACK_ROLLERS):
            y = y0 + (y1 - y0) * k / (TRACK_ROLLERS - 1)
            rollers.append(('roller%d' % (k + 1), 'track_roller.step',
                            (bottom, y), rt))
        rollers.append(('carrier1', 'carrier_roller.step',
                        (r * math.cos(first) - rail - rc - gap, 0.5 * y_idler),
                        rc))
    offset = SOIL_RAIL_OFFSET / 1000.0
    for name, filename, center, radius in rollers:
        m, radial, axial = roller_mass(radius * 1000.0)
        treads = [sphere(radius, (0.0, 0.0, side * offset))
                  for side in (-1.0, 1.0)]
        bodies.append({
            'name': name,
            'geometry': [{'type': 'cad', 'filename': filename}],
            'collision_geometry': (treads if soil else
                                   [{'type': 'cad', 'material': 'steel'}]),
            'center_of_mass': [0.0, 0.0, 0.0],
            'inertia': inertia(number(m), number(radial), number(radial),
                               number(axial))})
    if frame:
        # the frame's origin at the sprocket's axle; its axles' frames
        # turn the wheels' axes (z) onto y
        origin = place((0.0, 0.0, 0.0))

        def axle(name, center):
            t = place((center[0], center[1], 0.0))
            return {'name': name,
                    'T': [number(t[k] - origin[k]) for k in range(3)],
                    'R': WHEEL_FRAME}
        frames = [axle('sprocket_axle', (0.0, 0.0)),
                  axle('idler_axle', (0.0, y_idler))]
        frames += [axle(name + '_axle', center)
                   for name, _, center, _ in rollers]
        length = -y_idler
        bodies.append({
            'name': 'frame', 'frames': frames, 'geometry': [],
            'center_of_mass': [number(0.5 * length), 0.0, 0.0],
            'inertia': inertia(FRAME_MASS, number(FRAME_MASS * 0.5),
                               number(FRAME_MASS * length ** 2 / 12.0),
                               number(FRAME_MASS * length ** 2 / 12.0))})
        # slides forward and down, light bodies between joints
        for name in ('carriage_x', 'carriage_z'):
            bodies.append({'name': name, 'geometry': [],
                           'center_of_mass': [0.0, 0.0, 0.0],
                           'inertia': inertia(1.0, 0.01, 0.01, 0.01)})

    # a link of steel, from the volumes of its solids, as in the tests
    link_contact = contact
    if soil:
        link_contact = [sphere(BUSHING_RADIUS / 1000.0)]
        for side in (-1.0, 1.0):
            link_contact.append({
                'type': 'box', 'material': 'steel',
                'half_extents': [number(0.5 * p),
                                 number(SOIL_RAIL_HALF_WIDTH / 1000.0), rail],
                'pose': {'T': [number(0.5 * p), side * offset, 0.0]}})
    for i in range(links):
        bodies.append({
            'name': 'link%d' % (i + 1),
            'frames': [{'name': 'pin', 'T': [p, 0.0, 0.0]}],
            'geometry': [{'type': 'cad', 'filename': 'track_link.step'}],
            'collision_geometry': link_contact,
            'center_of_mass': [0.0974, 0.0, 0.0359],
            'inertia': inertia(38.06, 0.75, 0.15, 0.8)})

    # the first link free, at the sprocket's first bushing; each next one
    # turned about the previous one's pin, and the last pinned to the first
    x = direction(0)
    first_link = [[x[0], 0.0, x[1]], [x[1], 0.0, -x[0]], [0.0, 1.0, 0.0]]
    first_link = [[sum(WHEEL_FRAME[i][m] * first_link[m][k]
                       for m in range(3)) for k in range(3)]
                  for i in range(3)]
    first_bushing = place((bushings[0][0], bushings[0][1], 0.0))
    carrier = 'frame' if frame else 'fixed'
    joints = [
        {'name': 'sprocket_joint', 'type': 'revolute',
         'parent': [[carrier], 'sprocket_axle'],
         'child': [['sprocket'], 'origin'],
         'axis': [0.0, 0.0, 1.0],
         # a seat at the first bushing
         'initial_displacement': number(first)},
        {'name': 'idler_joint', 'type': 'revolute',
         'parent': [[carrier], 'idler_axle'], 'child': [['idler'], 'origin'],
         'axis': [0.0, 0.0, 1.0]},
        {'name': 'link1_joint', 'type': 'float',
         'parent': [['fixed'], 'origin'], 'child': [['link1'], 'origin'],
         'initial_displacement': {
             'T': [number(v) for v in first_bushing],
             'R': [[number(v) for v in row] for row in first_link]}},
    ]
    if not soil:
        joints.insert(2, {'name': 'ground_joint', 'type': 'rigid',
                          'parent': [['fixed'], 'origin'],
                          'child': [['ground'], 'origin']})
    if frame:
        joints += [
            {'name': 'slide_x', 'type': 'prismatic',
             'parent': [['fixed'], 'frame_start'],
             'child': [['carriage_x'], 'origin'], 'axis': [1.0, 0.0, 0.0]},
            {'name': 'slide_z', 'type': 'prismatic',
             'parent': [['carriage_x'], 'origin'],
             'child': [['carriage_z'], 'origin'], 'axis': [0.0, 0.0, 1.0]},
            {'name': 'pitch', 'type': 'revolute',
             'parent': [['carriage_z'], 'origin'],
             'child': [['frame'], 'origin'], 'axis': [0.0, 1.0, 0.0]}]
        for name, _, _, _ in rollers:
            joints.append({'name': name + '_joint', 'type': 'revolute',
                           'parent': [['frame'], name + '_axle'],
                           'child': [[name], 'origin'],
                           'axis': [0.0, 0.0, 1.0]})
    for i in range(1, links + 1):
        a, b = direction(i - 1), direction(i % links)
        # about the link's y, the sprocket's z: positive into the loop
        turn = math.atan2(a[0] * b[1] - a[1] * b[0], a[0] * b[0] + a[1] * b[1])
        child = 'link%d' % (i % links + 1)
        joints.append({
            'name': ('%s_joint' % child) if i < links else 'closing_joint',
            'type': 'revolute',
            'parent': [['link%d' % i], 'pin'], 'child': [[child], 'origin'],
            'axis': [0.0, 1.0, 0.0],
            'initial_displacement': number(turn)})

    steel = {'name': 'steel', 'youngs_modulus': LOOP_E, 'poisson_ratio': 0.3,
             'restitution': 0.3, 'friction': 0.3}
    force_modules = [{'name': 'contact', 'type': 'contact',
                      'materials': [steel]}]
    relative_error, absolute_error = 1e-6, 1e-9
    if soil:
        steel['contact_stiffness'] = SOIL_CONTACT_STIFFNESS
        force_modules.append({
            'name': 'soil', 'type': 'track_soil',
            'soil': {'name': 'dry_sand', 'wall_friction': 0.24},
            'shoes': ['link%d' % (i + 1) for i in range(links)],
            'shoe_center': [number(0.5 * p), 0.0,
                            number((SHOE_TOP + SHOE_THICKNESS +
                                    GROUSER_HEIGHT) / 1000.0)],
            'shoe_normal': [0.0, 0.0, 1.0], 'shoe_along': [1.0, 0.0, 0.0],
            'shoe_length': number(p),
            'shoe_width': number(2.0 * SHOE_HALF_WIDTH / 1000.0),
            'grouser_height': number(GROUSER_HEIGHT / 1000.0)})
    elif frame:
        # Archard wear of hardened steel on steel, K / H: K = 1e-4 over
        # H = 5 GPa
        steel['wear_coefficient'] = 2e-14
        # tighter tolerances than these only chase the chatter of the contacts,
        # and take the integrator far longer
        relative_error, absolute_error = 1e-4, 1e-4
        force_modules[0]['friction_velocity'] = 0.03
        to_bushing = rail  # from a rail to the bushings' line
        force_modules.append({
            'name': 'track', 'type': 'track', 'frame': 'frame',
            'links': ['link%d' % (i + 1) for i in range(links)],
            'guides': [name for name, _, _, _ in rollers],
            'up': [0.0, 0.0, -1.0], 'lateral': [0.0, 1.0, 0.0],
            'spans': [
                {'name': 'front', 'supports': ['idler', 'carrier1'],
                 'radii': [number(r), number(rc + gap + to_bushing)]},
                {'name': 'rear', 'supports': ['carrier1', 'sprocket'],
                 'radii': [number(rc + gap + to_bushing), number(r)]}]})
    soil_description = (
        'A track of %d links on dry sand (the track_soil force module), the '
        'track frame of half of a 20 t excavator (%.0f kg) free to move '
        'forward, up and down and to pitch, the sprocket driving it forward '
        'at %g rad/s by a PID controller. The STEP files are drawn; the contacts are on '
        'primitive shapes: spheres at the bushings, on spheres as the wheels\' '
        'treads and between pins on them, and boxes as the rails under '
        'spheres as the rollers\' treads. x forward, y to the right, z down. Written by '
        'scripts/make_track_step.py.' % (links, FRAME_MASS, LOOP_SPEED))
    description = (
        'A track loop of %d links (examples/cad/track_link.step) around the '
        'drive sprocket (sprocket.step) and the idler (idler.step), resting '
        'on the ground, the sprocket driven at %g rad/s by a PID controller: '
        'contact on the exact STEP surfaces. ' % (links, LOOP_SPEED))
    if frame:
        description += (
            'The wheels, %d track rollers (track_roller.step) and a carrier '
            'roller (carrier_roller.step) are on the track frame of half of '
            'a 20 t excavator (%.0f kg), free to move forward, up and down '
            'and to pitch, so the sprocket drives it along the ground. The '
            'contacts wear (Archard), and the track module logs the sag of '
            'the top spans, the misguide of the links and the side force on '
            'the flanges. ' % (TRACK_ROLLERS, FRAME_MASS))
    description += (
        'A benchmark, not run in the regular tests. x forward, y to the '
        'right, z down: the axles along y, the idler in front of the '
        'sprocket. Written by scripts/make_track_step.py.')
    if soil:
        description = soil_description
    if frame:
        model_frames = [{'name': 'frame_start',
                         'T': [number(v) for v in place((0.0, 0.0, 0.0))]}]
    else:
        # the wheels' frames: their axes (z) along the model's y
        model_frames = [
            {'name': 'sprocket_axle',
             'T': [number(v) for v in place((0.0, 0.0, 0.0))],
             'R': WHEEL_FRAME},
            {'name': 'idler_axle',
             'T': [number(v) for v in place((0.0, y_idler, 0.0))],
             'R': WHEEL_FRAME}]
    model = {
        'comment': description,
        'model_parameters': {
            'version': '0.2',
            'units': {'angle': 'radians', 'length': 'meters',
                      'mass': 'kilograms', 'time': 'seconds'},
            'gravity': [0.0, 0.0, 9.81],
            'include_directories': ['.'],
            'enable_contact_detection': True,
            'solvers': [{'type': 'rnca'}],
            'integrators': [{'type': 'ode113',
                             'relative_error': relative_error,
                             'absolute_error': absolute_error,
                             'time_span': SOIL_TIME if soil else LOOP_TIME,
                             'time_step': 0.01}]},
        'frames': model_frames,
        'assemblies': [{
            'name': 'track',
            'bodies': bodies,
            'joints': joints,
            'sensors': [{'type': 'joint', 'name': 'sprocket_speed',
                         'joint': 'sprocket_joint', 'quantity': 'velocity'}],
            'controllers': [{'type': 'pid', 'name': 'sprocket_drive',
                             'kp': SOIL_DRIVE_KP if soil else 500.0,
                             'ki': SOIL_DRIVE_KI if soil else 1000.0,
                             'kd': 0.0,
                             # forward, towards the idler
                             'target': -LOOP_SPEED if soil else LOOP_SPEED,
                             'sensor': 'sprocket_speed',
                             'joint': 'sprocket_joint'}],
            'force_modules': force_modules}]}
    with open(path, 'w') as f:
        json.dump(model, f, indent=2)
        f.write('\n')


if __name__ == '__main__':
    main()
