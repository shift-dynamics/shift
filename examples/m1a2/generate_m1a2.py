#!/usr/bin/env python3
"""writes the M1A2 Abrams tank models of this directory, and the STEP parts
they draw and touch with:

* m1a2.json: contact on primitive shapes (boxes, spheres and thin
  ellipsoids), the STEP parts only drawn;
* m1a2_step.json: contact on the exact surfaces of the STEP parts;
* track_link.step, sprocket.step, road_wheel.step, return_roller.step and
  road_arm.step.

The hull, turret and gun, the road wheels and the idlers are drawn with the
meshes of the source model (see readme.rst), split by
shift/scripts/split_vehicle_gltf.py with shift/scripts/vehicle_splits/m1a2.json.
The positions of the road wheels, sprockets and idlers were measured from those
meshes. The overall dimensions match the tank's (hull 7.8 m long, 3.67 m wide
over the skirts, 9.65 m long with the gun forward).

All masses, inertias, torsion bar rates, damper rates, contact properties
and the shape of the track are estimates, not data of the real tank: only the
combat weight (62 t), the road wheel diameter (25 in) and the track width
(635 mm) are published figures.

Each side: seven road wheels, each on a trailing road arm whose pivot is a
revolute joint with a torsion bar (a linear spring with a preload that holds
the arm at its design angle under the static load) and, at stations 1, 2
and 7, a rotary damper; a drive sprocket at the rear, an idler at the front,
two return rollers under the top run, and a single pin track of N links of
190 mm pitch, each link on a revolute joint to the next. The idler's
position along the hull is solved so that the loop of links closes exactly
(the tank adjusts its idler to tension the track). A PID controller holds the
speed of each sprocket at the target of a schedule: the tank settles, drives
away and turns right by slowing its right track.

Vehicle (SAE) coordinates: x forward, y right, z down, in meters, with the
origin on the ground midway between the sprocket and idler axles.

Usage: generate_m1a2.py [output directory] (default: this directory)
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'scripts'))
import make_track_step as track_step  # noqa: E402

SCRIPT = 'examples/m1a2/generate_m1a2.py'

# ---------------------------------------------------------------------------
# measured from the meshes of the source (vehicle coordinates, m)

# road wheel centers along x, stations 1 (front) to 7; the two sides are
# staggered, as the torsion bars of the two sides pass each other across the
# hull
ROAD_WHEEL_X = {
    'right': [1.9607, 1.1093, 0.4205, -0.2949, -0.9995, -1.7122, -2.4766],
    'left': [2.1657, 1.3077, 0.5665, -0.1372, -0.8522, -1.5767, -2.2871],
}
TRACK_Y = {'right': 1.40, 'left': -1.40}   # middle of each track
SPROCKET_CENTER = (-3.0846, -0.851)        # x, z
IDLER_CENTER = (3.0858, -0.832)            # x, z; x is solved for closure
# mesh centers (body origins of the split meshes): the meshes are drawn
# about the bodies' origins
TREAD_RADIUS = 0.3164                      # road wheels and idlers

# ---------------------------------------------------------------------------
# the track, mm (link coordinates: x along the track from the link's pin to
# the next one's, y along the pins, z out of the loop)

PITCH = track_step.PITCH                   # 190 mm, as the sprocket's
CONNECTOR_RADIUS = track_step.BUSHING_RADIUS  # 29.5: sits in the seats
CONNECTOR_Y = (252.5, 317.5)               # the end connectors, each side
PIN_RADIUS = 14.0                          # drawn only, between the blocks
BLOCK_X = (15.0, 175.0)                    # the block between the pins
BLOCK_HALF_WIDTH = 245.0
BLOCK_INNER = -40.0                        # the face the wheels run on
BLOCK_OUTER = 35.0
PAD_OUTER = 53.0                           # rubber pad, 18 mm
PAD_HALF_WIDTH = 317.5                     # 635 mm track
GUIDE_X = (55.0, 135.0)                    # the center guide horn
GUIDE_HALF_WIDTH = 35.0
GUIDE_TIP = -120.0

# wheels, mm: dual tires either side of the guide horns
TIRE_Y = (60.0, 230.0)
ROAD_WHEEL_HUB = 150.0
SPROCKET_RING_Y = (255.0, 315.0)           # on the end connectors
SPROCKET_HUB = 150.0
ROLLER_RADIUS = 150.0
ROLLER_TIRE_Y = (60.0, 200.0)
ROLLER_HUB = 50.0
CLEARANCE = 1.0                            # of the wheels off the links

# road arms
ARM_LENGTH = 0.45                          # pivot to wheel axle, m
ARM_ANGLE = math.radians(25.0)             # below the pivot, at design
ARM_INBOARD = 0.28                         # pivot inboard of the track, m
ARM_HALF_HEIGHT = 50.0                     # mm
ARM_HALF_THICKNESS = 40.0                  # mm
GROUND_GAP = 0.001                         # under the pads at the start, m
RETURN_ROLLERS = (0.35, 0.65)              # along the top run, front to rear

# ---------------------------------------------------------------------------
# estimates: masses (kg) and inertias (kg m^2) about each body's center of
# mass, along the vehicle's axes

TOTAL_MASS = 62000.0                       # combat weight, published
LINK_MASS = 45.0
ROAD_WHEEL_MASS = 180.0
ARM_MASS = 120.0
SPROCKET_MASS = 350.0
IDLER_MASS = 180.0
ROLLER_MASS = 25.0
# over the middle of the road wheels (the mean of their positions along x,
# -0.2 m), so that the torsion bars, all alike, share the load evenly
HULL_CENTER_OF_MASS = [-0.20, 0.0, -1.10]
HULL_BOX = (7.9, 3.66, 2.0)                # the box of the hull's inertia

# suspension: a wheel rate that gives the hull a bounce of about 1.3 Hz
WHEEL_RATE = 2.6e5                         # N/m at the wheel
DAMPED_STATIONS = (1, 2, 7)
DAMPER_RATE = 4.5e4                        # N s/m at the wheel

# drive and route
SPROCKET_KP = 1.2e5                        # N m per rad/s
SPROCKET_KI = 2.4e5
SETTLE_TIME = 1.0                          # s, the sprockets held still
SPEED = 1.5                                # m/s
ACCELERATE = (1.0, 3.0)                    # s, up to SPEED
TURN = (4.0, 5.0, 6.0, 7.0)                # s, the right track slowed and back
TURN_RATIO = 0.5                           # of the right track's speed
TIME_SPAN = 7.0

# contact
STEEL_E = 2e9           # a hundredth of steel's, as examples/cad/track_frame
RUBBER_E = 1e8          # the road wheels' tires
PAD_E = 5e7             # the track's rubber pads
GROUND_E = 5e7          # firm ground
BOX_STIFFNESS = 2e7     # per corner of the boxes of the primitive model, N/m
FRICTION_VELOCITY = 0.1

TAN = (0.55, 0.47, 0.33)
STEEL = (0.35, 0.35, 0.37)
RUBBER = (0.08, 0.08, 0.08)


def number(x):
    return float('%.12g' % x)


def inertia(mass, ixx, iyy, izz):
    return {'mass': number(mass), 'Ixx': number(ixx), 'Iyy': number(iyy),
            'Izz': number(izz), 'Ixy': 0.0, 'Iyz': 0.0, 'Izx': 0.0}


def box_inertia(mass, a, b, c):
    """of a box of sides a, b, c along x, y, z"""
    return inertia(mass, mass * (b * b + c * c) / 12.0,
                   mass * (a * a + c * c) / 12.0,
                   mass * (a * a + b * b) / 12.0)


def wheel_inertia(mass, radius, width):
    """of a disc about y"""
    radial = mass * (3.0 * radius * radius + width * width) / 12.0
    return inertia(mass, radial, 0.5 * mass * radius * radius, radial)


# ---------------------------------------------------------------------------
# STEP parts, in the coordinates of their bodies: x forward, y right, z down
# (the axles along y). Profiles are in (x, z), extruded along -y from y_top.

def along_y(name, loops, y_top, width, rgb):
    return (name,
            lambda s, n: track_step.extrude(s, n, loops, (0.0, y_top, 0.0),
                                            (1.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                                            width),
            rgb)


def both_sides(name, loops, y0, y1, rgb):
    """the solid from y0 to y1 and its mirror image in y"""
    return [along_y(name + ' right', loops, y1, y1 - y0, rgb),
            along_y(name + ' left', loops, -y0, y1 - y0, rgb)]


def rectangle(x0, x1, z0, z1):
    return track_step.polygon_loop([(x0, z0), (x1, z0), (x1, z1), (x0, z1)])


def disc(radius):
    return [track_step.circle_loop((0.0, 0.0), radius, True)]


def write_step(out):
    def write(name, comment, solids):
        track_step.write(os.path.join(out, name + '.step'), name,
                         comment + ' In millimeters. Written by %s.' % SCRIPT,
                         solids, SCRIPT)

    write('track_link',
          'One link of a single pin track like the M1 tank\'s (an estimate '
          'of its shape): %.0f mm pitch, %.0f mm wide. The link\'s pin is the '
          'y axis, the next link\'s at x = %.0f mm, z out of the track loop. '
          'A block between the pins that the wheels run on (its face at z = '
          '%.0f mm), a rubber pad on the ground side, a center guide horn '
          'between the dual wheels and an end connector at each end of the '
          'pin (%.0f mm diameter), which the sprocket drives.' %
          (PITCH, 2 * PAD_HALF_WIDTH, PITCH, BLOCK_INNER,
           2 * CONNECTOR_RADIUS),
          [along_y('block', [rectangle(BLOCK_X[0], BLOCK_X[1], BLOCK_INNER,
                                       BLOCK_OUTER)],
                   BLOCK_HALF_WIDTH, 2 * BLOCK_HALF_WIDTH, STEEL),
           along_y('pad', [rectangle(BLOCK_X[0], BLOCK_X[1], BLOCK_OUTER,
                                     PAD_OUTER)],
                   PAD_HALF_WIDTH, 2 * PAD_HALF_WIDTH, RUBBER),
           along_y('guide', [rectangle(GUIDE_X[0], GUIDE_X[1], GUIDE_TIP,
                                       BLOCK_INNER)],
                   GUIDE_HALF_WIDTH, 2 * GUIDE_HALF_WIDTH, STEEL),
           along_y('pin', [track_step.circle_loop((0.0, 0.0), PIN_RADIUS,
                                                  True)],
                   CONNECTOR_Y[0], 2 * CONNECTOR_Y[0], STEEL)] +
          both_sides('connector',
                     [track_step.circle_loop((0.0, 0.0), CONNECTOR_RADIUS,
                                             True)],
                     CONNECTOR_Y[0], CONNECTOR_Y[1], STEEL))

    teeth, pitch_radius, tip_radius = track_step.sprocket_profile()
    write('sprocket',
          'Drive sprocket: two rings of %d teeth on the end connectors of a '
          '%.0f mm pitch track, engaging every second tooth (as '
          'examples/cad/sprocket.step); pitch circle radius %.1f mm, tip '
          'circle %.1f mm, the rings %.0f to %.0f mm either side of the '
          'middle on a hub of %.0f mm radius. Axis along y.' %
          (track_step.TEETH, PITCH, pitch_radius, tip_radius,
           SPROCKET_RING_Y[0], SPROCKET_RING_Y[1], SPROCKET_HUB),
          both_sides('ring', [teeth], SPROCKET_RING_Y[0], SPROCKET_RING_Y[1],
                     TAN) +
          [along_y('hub', disc(SPROCKET_HUB), SPROCKET_RING_Y[0],
                   2 * SPROCKET_RING_Y[0], TAN)])

    write('road_wheel',
          'Dual road wheel (and idler) of the contact of m1a2_step.json: two '
          'tires of %.1f mm radius, %.0f to %.0f mm either side of the '
          'middle, either side of the track\'s guide horns, on a hub of %.0f '
          'mm radius. Axis along y.' %
          (TREAD_RADIUS * 1000.0, TIRE_Y[0], TIRE_Y[1], ROAD_WHEEL_HUB),
          both_sides('tire', disc(TREAD_RADIUS * 1000.0), TIRE_Y[0],
                     TIRE_Y[1], RUBBER) +
          [along_y('hub', disc(ROAD_WHEEL_HUB), TIRE_Y[0], 2 * TIRE_Y[0],
                   TAN)])

    write('return_roller',
          'Dual return roller under the top run of the track: two tires of '
          '%.0f mm radius, %.0f to %.0f mm either side of the middle, on a '
          'hub of %.0f mm radius under the guide horns. Axis along y.' %
          (ROLLER_RADIUS, ROLLER_TIRE_Y[0], ROLLER_TIRE_Y[1], ROLLER_HUB),
          both_sides('tire', disc(ROLLER_RADIUS), ROLLER_TIRE_Y[0],
                     ROLLER_TIRE_Y[1], RUBBER) +
          [along_y('hub', disc(ROLLER_HUB), ROLLER_TIRE_Y[0],
                   2 * ROLLER_TIRE_Y[0], TAN)])

    # the arm from its pivot (the origin) back and down to the wheel's axle
    length = ARM_LENGTH * 1000.0
    wheel = (-length * math.cos(ARM_ANGLE), length * math.sin(ARM_ANGLE))
    write('road_arm',
          'Road arm: a bar of %.0f mm from its pivot, the origin, back and '
          '%.0f degrees down to the axle of its road wheel, %.0f mm high and '
          '%.0f mm thick about y = 0. Axis of the pivot along y.' %
          (length, math.degrees(ARM_ANGLE), 2 * ARM_HALF_HEIGHT,
           2 * ARM_HALF_THICKNESS),
          [along_y('arm', [stadium((0.0, 0.0), wheel, ARM_HALF_HEIGHT)],
                   ARM_HALF_THICKNESS, 2 * ARM_HALF_THICKNESS, TAN)])
    return pitch_radius / 1000.0


def stadium(a, b, h):
    """the outline of a bar of half height h from a to b, with round ends,
    counterclockwise"""
    d = track_step.sub(b, a)
    t = track_step.scale(d, 1.0 / track_step.norm(d))
    n = (-t[1], t[0])

    def at(p, s):
        return track_step.add(p, track_step.scale(n, s))
    # along the right side, around b, back along the left side, around a
    return [('line', at(a, -h)), ('arc', at(b, -h), b, True),
            ('line', at(b, h)), ('arc', at(a, h), a, True)]


# ---------------------------------------------------------------------------
# the track's path: in the plane of a side, (u, w) = (x, -z), x forward and w
# up, around the circles of the pins' path, counterclockwise: forward along
# the bottom, up around the idler, back along the top and down around the
# sprocket

def tangent_normal(c1, r1, c2, r2):
    """the angle of the outward normal of the tangent from circle 1 to circle
    2, both on its left"""
    d = track_step.sub(c2, c1)
    return track_step.angle(d) - math.acos((r1 - r2) / track_step.norm(d))


class Path:
    """the convex hull of circles in loop order: arcs joined by tangents"""

    def __init__(self, circles):
        circles = list(circles)
        while True:
            n = len(circles)
            normals = [tangent_normal(circles[i][0], circles[i][1],
                                      circles[(i + 1) % n][0],
                                      circles[(i + 1) % n][1])
                       for i in range(n)]
            spans = [(normals[i] - normals[i - 1]) % (2.0 * math.pi)
                     for i in range(n)]
            # a circle the tangents pass outside of turns backwards
            inside = [i for i in range(n) if spans[i] > 1.5 * math.pi]
            if not inside:
                break
            del circles[inside[0]]
        self.circles = circles
        self.pieces = []  # (length, kind, data)
        for i, (c, r, name) in enumerate(circles):
            start = normals[i - 1]
            self.pieces.append((r * spans[i], 'arc', (c, r, start, name)))
            c2, r2, _ = circles[(i + 1) % n]
            a = track_step.add(c, track_step.scale(
                (math.cos(normals[i]), math.sin(normals[i])), r))
            b = track_step.add(c2, track_step.scale(
                (math.cos(normals[i]), math.sin(normals[i])), r2))
            self.pieces.append((track_step.norm(track_step.sub(b, a)), 'line',
                                (a, b, name)))
        self.length = sum(p[0] for p in self.pieces)

    def start_of(self, name):
        """arc length to the start of the arc on the circle named"""
        s = 0.0
        for length, kind, data in self.pieces:
            if kind == 'arc' and data[3] == name:
                return s
            s += length
        raise KeyError(name)

    def line_after(self, name):
        for length, kind, data in self.pieces:
            if kind == 'line' and data[2] == name:
                return data[0], data[1]
        raise KeyError(name)

    def point(self, s):
        s %= self.length
        for length, kind, data in self.pieces:
            if s <= length:
                if kind == 'arc':
                    c, r, start, _ = data
                    a = start + s / r
                    return (c[0] + r * math.cos(a), c[1] + r * math.sin(a))
                a, b, _ = data
                t = s / length if length > 0.0 else 0.0
                return track_step.add(a, track_step.scale(
                    track_step.sub(b, a), t))
            s -= length
        return self.point(0.0)


def chain(path, start, links, pitch):
    """the pins of links links of chord pitch along the path from arc length
    start; returns their arc lengths and that of the next pin, which closes
    the loop when it equals start + path.length"""
    s_values = [start]
    for _ in range(links):
        q = path.point(s_values[-1])
        lo, hi = s_values[-1] + 0.99 * pitch, s_values[-1] + 1.3 * pitch
        for _ in range(100):
            mid = 0.5 * (lo + hi)
            if track_step.norm(track_step.sub(path.point(mid), q)) < pitch:
                lo = mid
            else:
                hi = mid
        s_values.append(0.5 * (lo + hi))
    return s_values


def path_radius(tread):
    """the radius of the pins' path around a wheel of the tread radius, with
    the faces of the links wrapped around it CLEARANCE off the tread: the
    pins of a link of pitch p are at sqrt(a^2 + (p / 2)^2) from the center,
    with a the distance to the link's face plus the face's offset from the
    pins"""
    a = tread + (CLEARANCE - BLOCK_INNER) / 1000.0
    return math.hypot(a, PITCH / 2000.0)


def layout(side, pitch_radius):
    """the circles, pins and wheels of a side, its idler moved along x so
    that the loop closes"""
    p = PITCH / 1000.0
    wheel_path = path_radius(TREAD_RADIUS)
    # the wheels sit on the bottom run, the pads GROUND_GAP off the ground
    wheel_z = -(wheel_path + PAD_OUTER / 1000.0 + GROUND_GAP)
    wheels = [((x, -wheel_z), wheel_path, 'wheel%d' % (k + 1))
              for k, x in enumerate(ROAD_WHEEL_X[side])]

    def circles(idler_x):
        # counterclockwise: the sprocket, the wheels from the rear, the idler
        return ([((SPROCKET_CENTER[0], -SPROCKET_CENTER[1]), pitch_radius,
                  'sprocket')] + list(reversed(wheels)) +
                [((idler_x, -IDLER_CENTER[1]), wheel_path, 'idler')])

    def closure(idler_x, links):
        path = Path(circles(idler_x))
        # the first pin a little way onto the sprocket's arc
        start = path.start_of('sprocket') + pitch_radius * math.pi / 42.0
        s = chain(path, start, links, p)
        return s[-1] - (start + path.length), path, s

    links = int(round(Path(circles(IDLER_CENTER[0])).length / p))
    x0, x1 = IDLER_CENTER[0], IDLER_CENTER[0] + 0.01
    g0, g1 = closure(x0, links)[0], closure(x1, links)[0]
    for _ in range(50):
        x2 = x1 - g1 * (x1 - x0) / (g1 - g0)
        x0, g0 = x1, g1
        x1 = x2
        g1 = closure(x1, links)[0]
        if abs(g1) < 1e-13:
            break
    gap, path, s = closure(x1, links)
    pins = [path.point(v) for v in s[:-1]]
    closing = track_step.norm(track_step.sub(pins[0], pins[-1])) - p
    if abs(closing) > 1e-10:
        raise RuntimeError('the %s track does not close: %g m' %
                           (side, closing))
    # the return rollers under the top run, from the idler to the sprocket
    a, b = path.line_after('idler')
    d = track_step.sub(b, a)
    # into the loop: the left of the run, which is counterclockwise
    down = track_step.scale((-d[1], d[0]), 1.0 / track_step.norm(d))
    roller_path = (ROLLER_RADIUS + CLEARANCE - BLOCK_INNER) / 1000.0
    rollers = [track_step.add(track_step.add(a, track_step.scale(d, f)),
                              track_step.scale(down, roller_path))
               for f in RETURN_ROLLERS]
    return {'links': links, 'pins': pins, 'idler_x': x1, 'wheel_z': wheel_z,
            'rollers': rollers, 'path_length': path.length,
            'on_path': [c[2] for c in path.circles]}


# ---------------------------------------------------------------------------
# the models

def model(step_contact, sides, pitch_radius, out_name):
    p = PITCH / 1000.0
    bodies, joints, force_modules = [], [], []
    hull_frames = []
    sensors, controllers = [], []
    disabled = []

    def cad(filename, **extra):
        return dict({'type': 'cad', 'filename': filename}, **extra)

    def tread(radius, half_width, y, material):
        """a tire or tread: a thin ellipsoid about the axle, y, which touches
        boxes and spheres on closed forms (a cylinder takes the general
        search of GJK and EPA, many times slower)"""
        return {'type': 'ellipsoid',
                'semi_axes': [number(radius), number(half_width),
                              number(radius)],
                'material': material, 'pose': {'T': [0.0, number(y), 0.0]}}

    def sphere(radius, at, material):
        return {'type': 'sphere', 'radius': number(radius),
                'material': material, 'pose': {'T': [number(v) for v in at]}}

    def box(center, half, material, **extra):
        return dict({'type': 'box', 'material': material,
                     'half_extents': [number(v) for v in half],
                     'pose': {'T': [number(v) for v in center]}}, **extra)

    def dual(radius, tire_y, material):
        y0, y1 = tire_y[0] / 1000.0, tire_y[1] / 1000.0
        return [tread(radius, 0.5 * (y1 - y0), s * 0.5 * (y0 + y1),
                      material) for s in (-1.0, 1.0)]

    def wheel_contact(radius, tire_y):
        if step_contact:
            return [{'type': 'cad', 'material': 'rubber',
                     'fit_to': ['contact']}]
        return dual(radius, tire_y, 'rubber')

    def wheel_geometry(mesh_name):
        geometry = [{'type': 'mesh', 'filename': [mesh_name + '.glb']}]
        if step_contact:
            geometry.append(cad('road_wheel.step', name='contact',
                                active=False))
        return geometry

    # the links' contact on primitive shapes, in a link's coordinates
    mm = 0.001
    block_center = (0.5 * (BLOCK_X[0] + BLOCK_X[1]) * mm, 0.0,
                    0.5 * (BLOCK_INNER + BLOCK_OUTER) * mm)
    block_half = (0.5 * (BLOCK_X[1] - BLOCK_X[0]) * mm, BLOCK_HALF_WIDTH * mm,
                  0.5 * (BLOCK_OUTER - BLOCK_INNER) * mm)
    connector_y = 0.5 * (CONNECTOR_Y[0] + CONNECTOR_Y[1]) * mm
    link_contact = [
        box(block_center, block_half, 'track_steel'),
        box((block_center[0], 0.0, 0.5 * (BLOCK_OUTER + PAD_OUTER) * mm),
            (block_half[0], PAD_HALF_WIDTH * mm,
             0.5 * (PAD_OUTER - BLOCK_OUTER) * mm), 'pad'),
        box((0.5 * (GUIDE_X[0] + GUIDE_X[1]) * mm, 0.0,
             0.5 * (GUIDE_TIP + BLOCK_INNER) * mm),
            (0.5 * (GUIDE_X[1] - GUIDE_X[0]) * mm, GUIDE_HALF_WIDTH * mm,
             0.5 * (BLOCK_INNER - GUIDE_TIP) * mm), 'track_steel'),
        sphere(CONNECTOR_RADIUS * mm, (0.0, -connector_y, 0.0),
               'track_steel'),
        sphere(CONNECTOR_RADIUS * mm, (0.0, connector_y, 0.0),
               'track_steel')]
    if step_contact:
        # one material for all of the link's surfaces: steel, whose
        # contact with the far softer ground is set by the ground's
        link_contact = [{'type': 'cad', 'material': 'track_steel'}]

    # the sprocket's contact on primitive shapes: a tread under the
    # connectors and a pin between each two seats, in the plane of the
    # rings, either side
    ring_y = 0.5 * (SPROCKET_RING_Y[0] + SPROCKET_RING_Y[1]) * mm
    ring_half = 0.5 * (SPROCKET_RING_Y[1] - SPROCKET_RING_Y[0]) * mm
    sprocket_tread = pitch_radius - (CONNECTOR_RADIUS + CLEARANCE) * mm
    teeth = track_step.TEETH
    sprocket_contact = []
    for side_y in (-ring_y, ring_y):
        sprocket_contact.append(tread(sprocket_tread, ring_half, side_y,
                                      'track_steel'))
        for k in range(teeth):
            a = math.pi * (2 * k + 1) / teeth
            sprocket_contact.append(sphere(
                track_step.SOIL_PIN_RADIUS * mm,
                (pitch_radius * math.cos(a), side_y,
                 pitch_radius * math.sin(a)), 'track_steel'))
    if step_contact:
        sprocket_contact = [{'type': 'cad', 'material': 'track_steel'}]

    # torsion bars: f = -(c0 + k q) on each arm's joint, q = 0 at design
    lever = ARM_LENGTH * math.cos(ARM_ANGLE)  # horizontal, pivot to axle
    torsion_rate = WHEEL_RATE * lever * lever
    damper_rate = DAMPER_RATE * lever * lever

    unsprung = 0.0
    side_masses = 0.0
    for side, lay in sides.items():
        side_masses += (lay['links'] * LINK_MASS +
                        7 * (ROAD_WHEEL_MASS + ARM_MASS) + SPROCKET_MASS +
                        IDLER_MASS + len(RETURN_ROLLERS) * ROLLER_MASS)
        unsprung += lay['links'] * LINK_MASS * 0.5 + 7 * ROAD_WHEEL_MASS
    hull_mass = TOTAL_MASS - side_masses
    # the static load of each wheel: the hull and what it carries above the
    # wheels, shared evenly, through the arm's lever
    station_load = (TOTAL_MASS - unsprung) * 9.81 / 14.0
    preload = -station_load * lever

    for side, lay in sides.items():
        y = TRACK_Y[side]
        sign = 1.0 if side == 'right' else -1.0
        c = side + '_'
        link_names = ['%slink%d' % (c, i + 1) for i in range(lay['links'])]

        def at(u, w):
            return [number(u), number(y), number(-w)]

        # the sprocket, turned so that a seat holds the first pin
        sx, sz = SPROCKET_CENTER
        first = lay['pins'][0]
        seat = math.atan2(-first[1] - sz, first[0] - sx)
        hull_frames.append({'name': c + 'sprocket_axle',
                            'T': [number(sx), number(y), number(sz)]})
        bodies.append({
            'name': c + 'sprocket',
            'geometry': [cad('sprocket.step')],
            'collision_geometry': sprocket_contact,
            'center_of_mass': [0.0, 0.0, 0.0],
            'inertia': wheel_inertia(SPROCKET_MASS, 0.32, 0.6)})
        joints.append({'name': c + 'sprocket_joint', 'type': 'revolute',
                       'parent': [['hull'], c + 'sprocket_axle'],
                       'child': [[c + 'sprocket'], 'origin'],
                       'axis': [0.0, 1.0, 0.0],
                       'initial_displacement': number(-seat)})
        sensors.append({'type': 'joint', 'name': c + 'sprocket_speed',
                        'joint': c + 'sprocket_joint',
                        'quantity': 'velocity'})
        controllers.append({'type': 'pid', 'name': c + 'sprocket_drive',
                            'kp': SPROCKET_KP, 'ki': SPROCKET_KI, 'kd': 0.0,
                            'target': 0.0, 'sensor': c + 'sprocket_speed',
                            'joint': c + 'sprocket_joint'})

        # the idler, on the hull where the loop closes
        hull_frames.append({'name': c + 'idler_axle',
                            'T': [number(lay['idler_x']), number(y),
                                  number(IDLER_CENTER[1])]})
        bodies.append({
            'name': c + 'idler', 'geometry': wheel_geometry(c + 'idler'),
            'collision_geometry': wheel_contact(TREAD_RADIUS, TIRE_Y),
            'center_of_mass': [0.0, 0.0, 0.0],
            'inertia': wheel_inertia(IDLER_MASS, TREAD_RADIUS, 0.46)})
        joints.append({'name': c + 'idler_joint', 'type': 'revolute',
                       'parent': [['hull'], c + 'idler_axle'],
                       'child': [[c + 'idler'], 'origin'],
                       'axis': [0.0, 1.0, 0.0]})

        # the road arms and wheels
        arm_wheel = [-ARM_LENGTH * math.cos(ARM_ANGLE), sign * ARM_INBOARD,
                     ARM_LENGTH * math.sin(ARM_ANGLE)]
        for k, x in enumerate(ROAD_WHEEL_X[side]):
            station = k + 1
            arm = '%sroad_arm%d' % (c, station)
            wheel = '%sroad_wheel%d' % (c, station)
            pivot = [x - arm_wheel[0], y - arm_wheel[1],
                     lay['wheel_z'] - arm_wheel[2]]
            hull_frames.append({'name': arm + '_pivot',
                                'T': [number(v) for v in pivot]})
            bodies.append({
                'name': arm, 'geometry': [cad('road_arm.step')],
                'frames': [{'name': 'axle',
                            'T': [number(v) for v in arm_wheel]}],
                'center_of_mass': [number(0.5 * arm_wheel[0]), 0.0,
                                   number(0.5 * arm_wheel[2])],
                'inertia': inertia(ARM_MASS, 0.5,
                                   ARM_MASS * ARM_LENGTH ** 2 / 12.0,
                                   ARM_MASS * ARM_LENGTH ** 2 / 12.0)})
            arm_joint = {'name': arm + '_joint', 'type': 'revolute',
                         'parent': [['hull'], arm + '_pivot'],
                         'child': [[arm], 'origin'],
                         'axis': [0.0, 1.0, 0.0],
                         # the torsion bar
                         'stiffness_coefficients': [number(preload),
                                                    number(torsion_rate)]}
            if station in DAMPED_STATIONS:
                arm_joint['damping_coefficients'] = [0.0,
                                                     number(damper_rate)]
            joints.append(arm_joint)
            bodies.append({
                'name': wheel, 'geometry': wheel_geometry(wheel),
                'collision_geometry': wheel_contact(TREAD_RADIUS, TIRE_Y),
                'center_of_mass': [0.0, 0.0, 0.0],
                'inertia': wheel_inertia(ROAD_WHEEL_MASS, TREAD_RADIUS,
                                         0.46)})
            joints.append({'name': wheel + '_joint', 'type': 'revolute',
                           'parent': [[arm], 'axle'],
                           'child': [[wheel], 'origin'],
                           'axis': [0.0, 1.0, 0.0]})

        # the return rollers
        for k, (u, w) in enumerate(lay['rollers']):
            roller = '%sreturn_roller%d' % (c, k + 1)
            hull_frames.append({'name': roller + '_axle', 'T': at(u, w)})
            contact = (
                [{'type': 'cad', 'material': 'rubber'}] if step_contact else
                dual(ROLLER_RADIUS * mm, ROLLER_TIRE_Y, 'rubber'))
            bodies.append({
                'name': roller, 'geometry': [cad('return_roller.step')],
                'collision_geometry': contact,
                'center_of_mass': [0.0, 0.0, 0.0],
                'inertia': wheel_inertia(ROLLER_MASS, ROLLER_RADIUS * mm,
                                         0.4)})
            joints.append({'name': roller + '_joint', 'type': 'revolute',
                           'parent': [['hull'], roller + '_axle'],
                           'child': [[roller], 'origin'],
                           'axis': [0.0, 1.0, 0.0]})

        # the links: the first free, each next one turned about the
        # previous one's pin, the last pinned to the first
        pins = lay['pins']
        n = len(pins)

        def direction(i):
            d = track_step.sub(pins[(i + 1) % n], pins[i])
            return track_step.scale(d, 1.0 / track_step.norm(d))

        for i, name in enumerate(link_names):
            bodies.append({
                'name': name,
                'frames': [{'name': 'pin', 'T': [p, 0.0, 0.0]}],
                'geometry': [cad('track_link.step')],
                'collision_geometry': link_contact,
                'center_of_mass': [number(0.5 * p), 0.0, 0.0],
                'inertia': box_inertia(LINK_MASS, 0.16, 0.635, 0.1)})
        d = direction(0)
        joints.append({
            'name': link_names[0] + '_joint', 'type': 'float',
            'parent': [['fixed'], 'origin'],
            'child': [[link_names[0]], 'origin'],
            'initial_displacement': {
                'T': at(pins[0][0], pins[0][1]),
                # x along the link, z out of the loop: (d_w, d_u) in (x, z)
                'R': [[number(d[0]), 0.0, number(d[1])], [0.0, 1.0, 0.0],
                      [number(-d[1]), 0.0, number(d[0])]]}})
        for i in range(1, n + 1):
            a, b = direction(i - 1), direction(i % n)
            turn = math.atan2(a[0] * b[1] - a[1] * b[0],
                              a[0] * b[0] + a[1] * b[1])
            child = link_names[i % n]
            joints.append({
                'name': ('%s_joint' % child) if i < n else c + 'closing_joint',
                'type': 'revolute',
                'parent': [[link_names[i - 1]], 'pin'],
                'child': [[child], 'origin'],
                'axis': [0.0, 1.0, 0.0], 'initial_displacement': number(turn)})
            disabled.append([link_names[i - 1], child])

        force_modules.append({
            'name': c + 'track', 'type': 'track', 'frame': 'hull',
            'links': link_names,
            'guides': ['%sroad_wheel%d' % (c, k + 1) for k in range(7)],
            'up': [0.0, 0.0, -1.0], 'lateral': [0.0, 1.0, 0.0],
            'center': y,
            'spans': [
                {'name': 'front', 'supports': [c + 'idler',
                                               c + 'return_roller1'],
                 'radii': [number(path_radius(TREAD_RADIUS)),
                           number((ROLLER_RADIUS + CLEARANCE - BLOCK_INNER) *
                                  mm)]},
                {'name': 'rear', 'supports': [c + 'return_roller2',
                                              c + 'sprocket'],
                 'radii': [number((ROLLER_RADIUS + CLEARANCE - BLOCK_INNER) *
                                  mm), number(pitch_radius)]}]})

    # the hull, turret and gun, floating
    bodies.insert(0, {
        'name': 'hull', 'geometry': [{'type': 'mesh',
                                      'filename': ['hull.glb']}],
        'frames': hull_frames,
        'center_of_mass': HULL_CENTER_OF_MASS,
        'inertia': box_inertia(hull_mass, *HULL_BOX)})
    joints.insert(0, {'name': 'hull_joint', 'type': 'float',
                      'parent': [['fixed'], 'origin'],
                      'child': [['hull'], 'origin']})

    # contact: steel on steel (a hundredth of steel's modulus, as the track
    # examples in examples/cad), the rubber of the wheels on the links, and
    # the rubber pads on firm ground
    materials = [
        {'name': 'track_steel', 'youngs_modulus': STEEL_E,
         'poisson_ratio': 0.3, 'restitution': 0.3, 'friction': 0.3,
         'contact_stiffness': 1e9},
        {'name': 'rubber', 'youngs_modulus': RUBBER_E, 'poisson_ratio': 0.45,
         'restitution': 0.3, 'friction': 0.5,
         'contact_stiffness': 1e8},
        {'name': 'pad', 'youngs_modulus': PAD_E, 'poisson_ratio': 0.45,
         'restitution': 0.1, 'friction': 0.8,
         'contact_stiffness': BOX_STIFFNESS},
        {'name': 'ground', 'youngs_modulus': GROUND_E, 'poisson_ratio': 0.3,
         'restitution': 0.1, 'friction': 0.8,
         'contact_stiffness': BOX_STIFFNESS}]
    ground_material = 'pad' if not step_contact else 'track_steel'
    force_modules.insert(0, {
        'name': 'contact', 'type': 'contact',
        'friction_velocity': FRICTION_VELOCITY,
        'materials': materials,
        'pairs': [{'materials': [ground_material, 'ground'],
                   'restitution': 0.1, 'friction': 0.8}],
        'disabled_pairs': disabled})

    # the route: held still while the tank settles, away to SPEED, then the
    # right track slowed to turn right and back
    w = -SPEED / pitch_radius  # the sprockets turn backwards to drive forward
    t0, t1 = ACCELERATE
    left = [0.0, 0.0, t0, 0.0, t1, w, TIME_SPAN, w]
    right = [0.0, 0.0, t0, 0.0, t1, w, TURN[0], w, TURN[1], TURN_RATIO * w,
             TURN[2], TURN_RATIO * w, TURN[3], w]
    for side, points in (('left', left), ('right', right)):
        controllers.append({
            'type': 'schedule', 'name': side + '_sprocket_speed_target',
            'interpolation': 'cubic',
            'points': [number(v) for v in points],
            'targets': ['controllers/%s_sprocket_drive/target' % side]})

    links = sum(lay['links'] for lay in sides.values())
    description = (
        'An M1A2 Abrams tank (%.0f t) on firm ground: seven road wheels a '
        'side on torsion bar road arms, dampers at stations 1, 2 and 7, a '
        'rear drive sprocket, a front idler, two return rollers and a single '
        'pin track a side (%d left and %d right links, %d in all), the '
        'sprockets on '
        'PID speed controllers. It settles for %g s, drives away to %g m/s '
        'and turns right. Contact on %s. The masses, springs, dampers and '
        'the track are estimates; see readme.rst. x forward, y right, z '
        'down. Written by %s.' %
        (TOTAL_MASS / 1000.0, sides['left']['links'],
         sides['right']['links'], links, SETTLE_TIME, SPEED,
         'the exact surfaces of the STEP parts' if step_contact else
         'primitive shapes (boxes, spheres and thin ellipsoids), the STEP '
         'parts only drawn', SCRIPT))
    return {
        'comment': description,
        'model_parameters': {
            'version': '0.2',
            'units': {'angle': 'radians', 'length': 'meters',
                      'mass': 'kilograms', 'time': 'seconds'},
            'gravity': [0.0, 0.0, 9.81],
            'include_directories': ['.'],
            'enable_contact_detection': True,
            'solvers': [{'type': 'rnca'}],
            'integrators': [{'type': 'ode113', 'relative_error': 1e-4,
                             'absolute_error': 1e-4,
                             'time_span': TIME_SPAN, 'time_step': 0.01}]},
        'frames': [{'name': 'light_1', 'T': [10.0, -10.0, -10.0]},
                   {'name': 'light_2', 'T': [-10.0, 10.0, -10.0]}],
        'lights': [
            {'name': 'light_1', 'type': 'point',
             'parent': ['fixed', 'light_1'], 'Ka': [0.4, 0.4, 0.4],
             'Kd': [0.6, 0.6, 0.6], 'Ks': [0.2, 0.2, 0.2]},
            {'name': 'light_2', 'type': 'point',
             'parent': ['fixed', 'light_2'], 'Ka': [0.6, 0.6, 0.6],
             'Kd': [0.6, 0.6, 0.6], 'Ks': [0.2, 0.2, 0.2]}],
        'collision_geometry': [{'name': 'ground', 'type': 'plane',
                                'material': 'ground', 'max_depth': 0.05}],
        'assemblies': [{
            'name': 'm1a2', 'bodies': bodies, 'joints': joints,
            'sensors': sensors, 'controllers': controllers,
            'force_modules': force_modules}]}


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(
        os.path.abspath(__file__))
    pitch_radius = write_step(out)
    sides = {side: layout(side, pitch_radius) for side in ('left', 'right')}
    for side, lay in sides.items():
        print('%s: %d links, idler at x = %.4f m (mesh %.4f), on the path: %s'
              % (side, lay['links'], lay['idler_x'], IDLER_CENTER[0],
                 ' '.join(lay['on_path'])))
    for name, step_contact in (('m1a2.json', False),
                               ('m1a2_step.json', True)):
        with open(os.path.join(out, name), 'w') as f:
            json.dump(model(step_contact, sides, pitch_radius, name), f,
                      indent=2)
            f.write('\n')


if __name__ == '__main__':
    main()
