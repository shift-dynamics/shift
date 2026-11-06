#!/usr/bin/env python3
"""Write the STEP files of a cam and its followers, for the tests of line
contact on curved faces, and a model of them: examples/cad/cam_nurbs.step and cam_extrusion.step
(the same cam, its side as rational B-spline surfaces and as surfaces of linear
extrusion of rational B-spline curves), roller.step and flat_follower.step.

The cam is an ellipse of semi-axes 40 mm and 25 mm, so that its radius of
curvature is known at every point, 20 mm wide. Each quarter of the ellipse is
an exact rational quadratic curve, and its own face. The axis of the cam is z,
its long axis x. The roller is 10 mm in radius and 20 mm long, and the follower
a flat plate, its face the plane x = 0, 40 mm by 20 mm. The model
examples/cad/cam_follower.json turns the cam against a sprung roller.

Usage: make_cam_step.py [output directory] (default examples/cad)
"""

import json
import math
import os
import sys

from make_track_step import Step, circle_loop, extrude, polygon_loop, write

SEMI_MAJOR = 40.0
SEMI_MINOR = 25.0
WIDTH = 20.0
ROLLER_RADIUS = 10.0
PLATE_HALF_HEIGHT = 20.0
PLATE_THICKNESS = 5.0
WEIGHTS = (1.0, math.sqrt(0.5), 1.0)
CONTACT_E = 2e9     # Young's modulus of the contact, a hundredth of steel's:
                    # steel's needs steps about 10 times smaller


def quarters():
    """the three control points of each quarter of the ellipse, in turn
    counterclockwise from the end of the long axis"""
    a, b = SEMI_MAJOR, SEMI_MINOR
    return [[(a, 0.0), (a, b), (0.0, b)],
            [(0.0, b), (-a, b), (-a, 0.0)],
            [(-a, 0.0), (-a, -b), (0.0, -b)],
            [(0.0, -b), (a, -b), (a, 0.0)]]


def rational_curve(step, points):
    weights = ','.join(repr(w) for w in WEIGHTS)
    return step.add(
        "(BOUNDED_CURVE()B_SPLINE_CURVE(2,(%s),.UNSPECIFIED.,.F.,.F.)"
        "B_SPLINE_CURVE_WITH_KNOTS((3,3),(0.,1.),.UNSPECIFIED.)CURVE()"
        "GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE((%s))"
        "REPRESENTATION_ITEM(''))" %
        (','.join('#%d' % step.point(p) for p in points), weights))


def rational_surface(step, control):
    """quadratic along the ellipse, straight across the cam"""
    rows = ','.join('(#%d,#%d)' % (step.point((p[0], p[1], 0.0)),
                                   step.point((p[0], p[1], WIDTH)))
                    for p in control)
    weights = ','.join('(%r,%r)' % (w, w) for w in WEIGHTS)
    return step.add(
        "(BOUNDED_SURFACE()B_SPLINE_SURFACE(2,1,(%s),.UNSPECIFIED.,.F.,.F.,"
        ".F.)B_SPLINE_SURFACE_WITH_KNOTS((3,3),(2,2),(0.,1.),(0.,1.),"
        ".UNSPECIFIED.)GEOMETRIC_REPRESENTATION_ITEM()"
        "RATIONAL_B_SPLINE_SURFACE((%s))REPRESENTATION_ITEM('')SURFACE())" %
        (rows, weights))


def cam(step, name, extrusion):
    """the solid of the cam: its side, four faces, between two planes"""
    z_axis = (0.0, 0.0, 1.0)
    parts = quarters()
    bottom_v = [step.add("VERTEX_POINT('',#%d)" %
                         step.point((c[0][0], c[0][1], 0.0))) for c in parts]
    top_v = [step.add("VERTEX_POINT('',#%d)" %
                      step.point((c[0][0], c[0][1], WIDTH))) for c in parts]
    up = []
    for i, c in enumerate(parts):
        line = step.add("LINE('',#%d,#%d)" % (
            step.point((c[0][0], c[0][1], 0.0)),
            step.add("VECTOR('',#%d,%s)" % (step.direction(z_axis),
                                            step.num(WIDTH)))))
        up.append(step.add("EDGE_CURVE('',#%d,#%d,#%d,.T.)" %
                           (bottom_v[i], top_v[i], line)))
    faces, bottom, top = [], [], []
    for i, c in enumerate(parts):
        j = (i + 1) % 4
        low = rational_curve(step, [(p[0], p[1], 0.0) for p in c])
        high = rational_curve(step, [(p[0], p[1], WIDTH) for p in c])
        bottom.append(step.add("EDGE_CURVE('',#%d,#%d,#%d,.T.)" %
                               (bottom_v[i], bottom_v[j], low)))
        top.append(step.add("EDGE_CURVE('',#%d,#%d,#%d,.T.)" %
                            (top_v[i], top_v[j], high)))
        if extrusion:
            surface = step.add(
                "SURFACE_OF_LINEAR_EXTRUSION('',#%d,#%d)" %
                (low, step.add("VECTOR('',#%d,%s)" % (step.direction(z_axis),
                                                      step.num(WIDTH)))))
        else:
            surface = rational_surface(step, c)
        oriented = [step.add("ORIENTED_EDGE('',*,*,#%d,.T.)" % bottom[i]),
                    step.add("ORIENTED_EDGE('',*,*,#%d,.T.)" % up[j]),
                    step.add("ORIENTED_EDGE('',*,*,#%d,.F.)" % top[i]),
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


def follower_model(path):
    """a cam turning on a flywheel, its roller on a slider pressed against it
    by a spring. The model's z is down, along the axes of the cam and roller;
    the slider runs along x"""
    def inertia(mass, ixx, iyy, izz):
        return {'mass': mass, 'Ixx': ixx, 'Iyy': iyy, 'Izz': izz,
                'Ixy': 0.0, 'Iyz': 0.0, 'Izx': 0.0}

    half = WIDTH / 2000.0
    start = (SEMI_MAJOR + ROLLER_RADIUS) / 1000.0
    # the spring is 20 mm short at the top of the cam, 5 mm at the bottom
    seat = start + 0.04
    steel = 'steel'
    model = {
        'model_parameters': {
            'version': '0.2',
            'units': {'angle': 'radians', 'length': 'meters',
                      'mass': 'kilograms', 'time': 'seconds'},
            'gravity': [0.0, 0.0, 9.81],
            'enable_contact_detection': True,
            'solvers': [{'type': 'rnca'}],
            'integrators': [{'type': 'ode113', 'relative_error': 1e-5,
                             'absolute_error': 1e-8, 'time_span': 0.7,
                             'time_step': 0.002}]},
        'assemblies': [{
            'name': 'cam_follower',
            'bodies': [
                {'name': 'cam',
                 'geometry': [{'type': 'cad', 'filename': 'cam_nurbs.step'}],
                 'collision_geometry': [{'type': 'cad', 'material': steel}],
                 # with a flywheel, so that it turns nearly evenly
                 'center_of_mass': [0.0, 0.0, half],
                 'inertia': inertia(5.0, 0.02, 0.02, 0.04)},
                {'name': 'slider', 'geometry': [],
                 'center_of_mass': [0.0, 0.0, 0.0],
                 'inertia': inertia(0.2, 1e-4, 1e-4, 1e-4)},
                {'name': 'roller',
                 'geometry': [{'type': 'cad', 'filename': 'roller.step'}],
                 'collision_geometry': [{'type': 'cad', 'material': steel}],
                 'center_of_mass': [0.0, 0.0, half],
                 'inertia': inertia(0.05, 2e-6, 2e-6, 2.5e-6)}],
            'joints': [
                {'name': 'cam_shaft', 'type': 'revolute',
                 'parent': [['fixed'], 'origin'], 'child': [['cam'], 'origin'],
                 'axis': [0, 0, 1], 'initial_velocity': 10.0},
                {'name': 'slide', 'type': 'prismatic',
                 'parent': [['fixed'], 'guide'],
                 'child': [['slider'], 'origin'], 'axis': [1, 0, 0]},
                {'name': 'roller_pin', 'type': 'revolute',
                 'parent': [['slider'], 'origin'],
                 'child': [['roller'], 'origin'], 'axis': [0, 0, 1]}],
            'force_modules': [
                {'type': 'contact', 'name': 'contact',
                 'materials': [{'name': steel, 'youngs_modulus': CONTACT_E,
                                'poisson_ratio': 0.3, 'restitution': 0.5,
                                'friction': 0.1}]},
                {'type': 'spring_damper', 'name': 'return_spring',
                 'frames': [['fixed', 'spring_seat'], ['slider', 'origin']],
                 'free_length': 0.06, 'stiffness': 5000.0, 'damping': 5.0}]}]}
    model['frames'] = [{'name': 'guide', 'T': [start, 0.0, 0.0]},
                       {'name': 'spring_seat', 'T': [seat, 0.0, 0.0]}]
    with open(path, 'w') as f:
        json.dump(model, f, indent=2)
        f.write('\n')


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else 'examples/cad'
    x, y = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)
    steel = (0.6, 0.62, 0.65)
    for name, extrusion in (('cam_nurbs', False), ('cam_extrusion', True)):
        write(os.path.join(directory, name + '.step'), name,
              'an elliptical cam, see scripts/make_cam_step.py',
              [('cam', lambda s, n, e=extrusion: cam(s, n, e), steel)])
    write(os.path.join(directory, 'roller.step'), 'roller',
          'a roller of a cam follower, see scripts/make_cam_step.py',
          [('roller', lambda s, n: extrude(
              s, n, [circle_loop((0.0, 0.0), ROLLER_RADIUS, True)],
              (0.0, 0.0, 0.0), x, y, WIDTH), steel)])
    h, t = PLATE_HALF_HEIGHT, PLATE_THICKNESS
    write(os.path.join(directory, 'flat_follower.step'), 'flat_follower',
          'a flat cam follower, see scripts/make_cam_step.py',
          [('plate', lambda s, n: extrude(
              s, n, [polygon_loop([(0.0, -h), (t, -h), (t, h), (0.0, h)])],
              (0.0, 0.0, 0.0), x, y, WIDTH), steel)])
    follower_model(os.path.join(directory, 'cam_follower.json'))


if __name__ == '__main__':
    main()
