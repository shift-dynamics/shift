#!/usr/bin/env python3
"""Render the figures of the theory pages on contact (doc/sphinx/contact) with
the viewer:

  python3 doc/figures/make_surface_figures.py [output directory]

(default doc/sphinx/contact/figures). It needs a release build with the
viewer and shift-doc-figures (doc/figures/surface_lines.cc), STEP support, a
display and ImageMagick; windows appear as it renders.

- topology.png: a part, its edges and its vertices
- shapes.png: the collision shapes given by their dimensions, from the
  polygons that shift-doc-figures writes from shift's own geometry
- meshes.png: a triangle mesh (the Stanford bunny) as a wireframe, and,
  about the shaded mesh, the collision shapes that shift fits to it (light purple, transparent), from
  shift-doc-figures fit
- terrain.png: the bicubic (a) and bilinear (b) terrain over one map,
  drawn by shift, with the samples of the map (black) and the boundaries of
  the cells (orange) from shift-doc-figures terrain, and terrain_section.png:
  the height, slope and second derivative of both along one row of samples
- broadphase.png: bunnies with their bounding spheres (purple) and the
  margins about them (grey), the candidate pairs of shift's broad phase (red)
  and, below them, the intervals of the enlarged spheres along the axis of the
  sweep, from shift-doc-figures broadphase
- boxtree.png: a track link with the boxes of its box tree, level by level,
  and stepcontact.png: the steps of the contact of two links, the second on
  the pin of the first: their root boxes, the faces whose boxes overlap, and
  the points of contact, all from shift-doc-figures cadpair
- surface_<type>.png: a face of each type of STEP surface, drawn by the viewer
  from the exact surface, with lines of constant u (blue) and of constant v
  (orange) inside its trims, its boundary (dark), the axes of its frame
  (x red, y green, z blue) and, for a B-spline surface, its control net
  (black)

The meshes, the model and the rendering are those of the figures of the
contact tests (test/contact/make_figures.py).
"""

import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

import numpy

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'test', 'contact'))
import make_figures as figures  # noqa: E402

BUILD = os.environ.get('SHIFT_BUILD', os.path.join(ROOT, 'build-release'))
TOOL = os.path.join(BUILD, 'doc', 'figures', 'shift-doc-figures')
CAD = os.path.join(ROOT, 'examples', 'cad')

U_COLOR = (0.15, 0.38, 0.85)
V_COLOR = (0.95, 0.55, 0.10)
EDGE_COLOR = (0.15, 0.15, 0.18)
AXIS_COLORS = {'x': (0.85, 0.10, 0.10), 'y': (0.10, 0.60, 0.15),
               'z': (0.10, 0.25, 0.85)}
NET_COLOR = (0.05, 0.05, 0.05)
SHAPE_COLOR = (0.55, 0.66, 0.80)

# (image, STEP file, surface type, which face of that type (an index, or
# 'largest'), view (azimuth, elevation), draw the frame, label)
SURFACES = [
    ('plane', 'flat_follower.step', 'plane', 'largest', (35, 30), True,
     'PLANE'),
    ('cylinder', 'cylinder_inch.step', 'cylinder', 0, (35, 25), True,
     'CYLINDRICAL_SURFACE'),
    ('cone', 'cone_apex.step', 'cone', 0, (35, 20), True, 'CONICAL_SURFACE'),
    ('sphere', 'tube_dome.step', 'sphere', 0, (35, -30), True,
     'SPHERICAL_SURFACE'),
    ('torus', 'torus_patch.step', 'torus', 0, (45, 30), True,
     'TOROIDAL_SURFACE'),
    ('revolution', 'revolution_patch.step', 'revolution', 0, (45, 15), True,
     'SURFACE_OF_REVOLUTION'),
    ('extrusion', 'gear_tooth.step', 'extrusion', 0, (-60, 35), False,
     'SURFACE_OF_LINEAR_EXTRUSION'),
    ('nurbs', 'rational_patch.step', 'nurbs', 0, (35, 35), False,
     'B_SPLINE_SURFACE_WITH_KNOTS'),
]

PANEL = (520, 420)


def tubes(lines, radius):
    mesh = figures.Mesh()
    for line in lines:
        figures.add_tube(mesh, numpy.array(line), radius)
    return mesh


def bounds(points):
    p = numpy.array(points)
    low, high = p.min(axis=0), p.max(axis=0)
    return 0.5 * (low + high), 0.5 * numpy.linalg.norm(high - low)


# the font of the text of the pages (the alabaster theme's font_family)
FONT = 'Georgia'


def label(image, text, target):
    subprocess.run(['magick', image, '-gravity', 'south', '-background',
                    'white', '-splice', '0x36', '-font', FONT, '-pointsize',
                    '22', '-fill', '#000000', '-annotate', '+0+6', text,
                    target], check=True)


def lettered(panels, work, name, target):
    """the panels in rows of four, each with its letter, (a), (b), ..., below
    it; the caption of the figure says what each one shows"""
    rows = []
    for r in range(0, len(panels), 4):
        lettered_row = []
        for i, panel in enumerate(panels[r:r + 4]):
            out = os.path.join(work, '%s_%d.png' % (name, r + i))
            label(panel, '(%s)' % chr(ord('a') + r + i), out)
            lettered_row.append(out)
        row = os.path.join(work, '%s_row%d.png' % (name, r))
        subprocess.run(['magick', *lettered_row, '+append', row], check=True)
        rows.append(row)
    subprocess.run(['magick', *rows, '-background', 'white', '-gravity',
                    'west', '-append', target], check=True)


def render(work, geometry, name, view, center, radius):
    figures.SIZE = PANEL
    model = figures.write_model(work, geometry, name)
    image = os.path.join(work, name + '.png')
    figures.render(model, image, view, center, radius)
    return image


def surface_figure(work, directory, entry):
    name, step, kind, which, view, frame, text = entry
    data = os.path.join(work, name + '.json')
    subprocess.run([TOOL, 'step', os.path.join(CAD, step), data, '8'],
                   check=True, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)
    with open(data) as file:
        faces = json.load(file)['faces']
    of_kind = [f for f in faces if f['surface'] == kind]
    if which == 'largest':
        face = max(of_kind, key=lambda f: numpy.ptp(
            numpy.array([p for line in f['boundary'] for p in line]),
            axis=0).sum())
    else:
        face = of_kind[which]
    every = [p for f in faces for line in f['boundary'] for p in line]
    center, scene = bounds(every)
    tube = 0.011 * scene
    link = os.path.join(work, step)
    if not os.path.exists(link):
        os.symlink(os.path.join(CAD, step), link)
    geometry = [{'type': 'cad', 'filename': step}]
    parts = [('u', tubes(face['u_lines'], tube), U_COLOR),
             ('v', tubes(face['v_lines'], tube), V_COLOR),
             ('edge', tubes(face['boundary'], 1.4 * tube), EDGE_COLOR)]
    if frame:
        origin = numpy.array(face['origin'])
        for axis in ('x', 'y', 'z'):
            arrow = figures.Mesh()
            figures.add_arrow(arrow, origin, face[axis], 0.45 * scene,
                              1.6 * tube)
            parts.append((axis, arrow, AXIS_COLORS[axis]))
        center, scene = bounds(every + [list(origin + 0.45 * scene *
                                             numpy.array(face[a]))
                                        for a in ('x', 'y', 'z')] +
                               [list(origin)])
    if 'control' in face:
        net = figures.Mesh()
        rows = [numpy.array(r) for r in face['control']]
        for row in rows:
            figures.add_tube(net, row, 0.6 * tube)
            for p in row:
                figures.add_ball(net, p, 2.2 * tube)
        for column in numpy.array(face['control']).transpose(1, 0, 2):
            figures.add_tube(net, column, 0.6 * tube)
        parts.append(('net', net, NET_COLOR))
        center, scene = bounds(every + [list(p) for r in rows for p in r])
    for part, mesh, color in parts:
        geometry.append(mesh.write(work, name + '_' + part, color, 1.0))
    image = render(work, geometry, name, view, center, 0.72 * scene)
    shutil.copy(image, os.path.join(directory, 'surface_%s.png' % name))
    print('surface_%s.png' % name, flush=True)


def topology_figure(work, directory):
    """a part with the boundaries of its faces (its edges) and its vertices"""
    step = 'track_link.step'
    data = os.path.join(work, 'topology.json')
    subprocess.run([TOOL, 'step', os.path.join(CAD, step), data, '2'],
                   check=True, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)
    with open(data) as file:
        model = json.load(file)
    every = [p for f in model['faces'] for line in f['boundary'] for p in line]
    center, scene = bounds(every)
    tube = 0.006 * scene
    os.symlink(os.path.join(CAD, step), os.path.join(work, step))
    edges = tubes([line for f in model['faces'] for line in f['boundary']],
                  tube)
    corners = figures.Mesh()
    for p in model['vertices']:
        figures.add_ball(corners, numpy.array(p), 3.0 * tube)
    geometry = [{'type': 'cad', 'filename': step},
                edges.write(work, 'topology_edges', (0.98, 0.82, 0.15), 1.0),
                corners.write(work, 'topology_vertices', AXIS_COLORS['x'], 1.0)]
    image = render(work, geometry, 'topology', (40, 35), center, 0.62 * scene)
    shutil.copy(image, os.path.join(directory, 'topology.png'))
    print('topology.png', flush=True)


def shapes_figure(work, directory):
    data = os.path.join(work, 'shapes.json')
    subprocess.run([TOOL, 'shapes', data], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    with open(data) as file:
        shapes = json.load(file)['shapes']
    panels = []
    for shape in shapes:
        mesh = figures.Mesh()
        points = []
        for polygon, normals in shape['polygons']:
            mesh.add(numpy.array(polygon), numpy.array(normals))
            points += polygon
        center, scene = bounds(points)
        geometry = [mesh.write(work, shape['name'], SHAPE_COLOR, 1.0)]
        image = render(work, geometry, shape['name'], (35, 25), center,
                       0.75 * scene)
        panels.append(image)
    # a plane: a square of it and its normal, out of the solid
    plane = figures.Mesh()
    outline = [numpy.array(p) for p in
               ([-0.3, -0.3, 0.0], [0.3, -0.3, 0.0], [0.3, 0.3, 0.0],
                [-0.3, 0.3, 0.0])]
    figures.add_plate(plane, outline, [0.0, 0.0, 1.0], 0.004)
    arrow = figures.Mesh()
    figures.add_arrow(arrow, numpy.zeros(3), [0.0, 0.0, -1.0], 0.25, 0.008)
    geometry = [plane.write(work, 'plane', SHAPE_COLOR, 1.0),
                arrow.write(work, 'plane_normal', AXIS_COLORS['z'], 1.0)]
    image = render(work, geometry, 'plane', (35, 25),
                   numpy.array([0.0, 0.0, -0.08]), 0.33)
    panels.append(image)
    lettered(panels, work, 'shapes', os.path.join(directory, 'shapes.png'))
    print('shapes.png', flush=True)


BUNNY = os.path.join(ROOT, 'examples', 'collision', 'stanford_bunny.obj')
# the bunny's up, +y of its file, drawn up: -z of the z-down convention
UPRIGHT = numpy.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])
WIRE_COLOR = (0.12, 0.12, 0.16)
VOLUME_COLOR = (0.72, 0.48, 1.0)
SOLID_COLOR = (0.70, 0.70, 0.70)
FITS = ['convex_hull', 'ellipsoid', 'sphere', 'box', 'cylinder', 'capsule']


def wireframe(meshes, radius):
    """the edges of the triangles of meshes, each a square prism"""
    mesh = figures.Mesh()
    every = []
    for data in meshes:
        v = numpy.array(data['vertices']).reshape(-1, 3) @ UPRIGHT.T
        f = numpy.array(data['faces']).reshape(-1, 3)
        every.append(v)
        edges = numpy.sort(numpy.concatenate([f[:, [0, 1]], f[:, [1, 2]],
                                              f[:, [2, 0]]]), axis=1)
        edges = numpy.unique(edges, axis=0)
        p, q = v[edges[:, 0]], v[edges[:, 1]]
        d = q - p
        d /= numpy.linalg.norm(d, axis=1)[:, None]
        a = numpy.cross(d, [0.31, 0.57, 0.76])
        a /= numpy.linalg.norm(a, axis=1)[:, None]
        b = numpy.cross(d, a)
        sides = [a, b, -a, -b]
        for k in range(4):
            n0, n1 = sides[k], sides[(k + 1) % 4]
            corners = [p + radius * n0, q + radius * n0, q + radius * n1,
                       p + radius * n1]
            normals = [n0, n0, n1, n1]
            for i in range(len(edges)):
                mesh.add([c[i] for c in corners], [n[i] for n in normals])
    return mesh, numpy.concatenate(every)


def shaded(meshes):
    """the triangles of meshes with smooth normals, area weighted"""
    mesh = figures.Mesh()
    for data in meshes:
        v = numpy.array(data['vertices']).reshape(-1, 3) @ UPRIGHT.T
        f = numpy.array(data['faces']).reshape(-1, 3)
        area = numpy.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
        n = numpy.zeros_like(v)
        for k in range(3):
            numpy.add.at(n, f[:, k], area)
        n /= numpy.linalg.norm(n, axis=1)[:, None]
        for t in f:
            mesh.add(v[t], n[t])
    return mesh


def meshes_figure(work, directory):
    os.symlink(BUNNY, os.path.join(work, 'stanford_bunny.obj'))
    model = figures.write_model(work, [{'type': 'mesh',
                                        'filename': ['stanford_bunny.obj']}],
                                'bunny_fit')
    with open(model) as file:
        fit = json.load(file)
    fit['assemblies'][0]['bodies'][0]['collision_geometry'] = [
        {'name': t, 'type': t} for t in FITS]
    with open(model, 'w') as file:
        json.dump(fit, file)
    data = os.path.join(work, 'bunny_fit_shapes.json')
    subprocess.run([TOOL, 'fit', model, data], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    with open(data) as file:
        fitted = json.load(file)
    shapes = {s['name']: s for s in fitted['shapes']}
    wire, points = wireframe(fitted['meshes'], 0.00012)
    wire_entry = wire.write(work, 'bunny_wire', WIRE_COLOR, 1.0)
    solid_entry = shaded(fitted['meshes']).write(work, 'bunny_solid',
                                                 SOLID_COLOR, 1.0)
    center, scene = bounds(points)
    # a close up of the head, where the triangles can be told apart
    top = points[numpy.argmin(points[:, 2])]
    head = points[numpy.linalg.norm(points - top, axis=1) < 0.06]
    head_center = head.mean(axis=0)
    image = render(work, [wire_entry], 'bunny_head', (60, 15), head_center,
                   0.022)
    close_up = image
    panels = []
    for name in ['mesh'] + FITS:
        geometry = [wire_entry if name == 'mesh' else solid_entry]
        if name != 'mesh':
            shape = figures.Mesh()
            for polygon, normals in shapes[name]['polygons']:
                shape.add(numpy.array(polygon) @ UPRIGHT.T,
                          numpy.array(normals) @ UPRIGHT.T)
            geometry.append(shape.write(work, 'fit_' + name, VOLUME_COLOR,
                                        0.3))
        image = render(work, geometry, 'bunny_' + name, (60, 15), center,
                       0.8 * scene)
        panels.append(image)
        if name == 'mesh':
            panels.append(close_up)
    lettered(panels, work, 'meshes', os.path.join(directory, 'meshes.png'))
    print('meshes.png', flush=True)


# heights of the control points of the terrain figures (m), a fixed pattern
TERRAIN_ROWS, TERRAIN_COLUMNS, TERRAIN_SPACING = 6, 8, 1.0
TERRAIN_SEED = 3
KNOT_COLOR = (0.95, 0.55, 0.10)


def terrain_heights():
    random = numpy.random.default_rng(TERRAIN_SEED)
    return 1.2 * random.random((TERRAIN_ROWS, TERRAIN_COLUMNS))


def terrain_model(work, interpolation):
    """a model of the terrain over the heights of TERRAIN_SEED, with the
    given interpolation"""
    from PIL import Image
    tiff = os.path.join(work, 'heights.tiff')
    # the default coordinate type: columns along x, rows along y, z down
    Image.fromarray(-terrain_heights().astype(numpy.float32),
                    mode='F').save(tiff)
    model = figures.write_model(work, [], 'terrain_' + interpolation)
    with open(model) as file:
        m = json.load(file)
    m['frames'] = [{'name': 'terrain', 'T': [0.0, 0.0, 0.0]}]
    m['terrain'] = [{'type': 'basic', 'name': 'map', 'filename': 'heights.tiff',
                     'parent': [['fixed'], 'terrain'],
                     'x_scale': TERRAIN_SPACING, 'y_scale': TERRAIN_SPACING,
                     'z_scale': 1.0, 'interpolation_type': interpolation}]
    with open(model, 'w') as file:
        json.dump(m, file)
    return model, m


def terrain_data(work, interpolation, per_cell):
    """the samples, control points and surface that shift computes"""
    model, _ = terrain_model(work, interpolation)
    data = os.path.join(work, 'terrain_%s.json' % interpolation)
    subprocess.run([TOOL, 'terrain', model, data, str(per_cell)], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    with open(data) as file:
        return json.load(file)


def terrain_figure(work, directory):
    """the terrain drawn by shift, with its samples (black) and the
    boundaries of its cells (orange) on it"""
    per_cell = 24
    panels = []
    for interpolation in ('bicubic', 'bilinear'):
        data = terrain_data(work, interpolation, per_cell)
        grid = numpy.array(data['surface'])
        p = grid[:, :, 0:3]
        scene = numpy.ptp(p.reshape(-1, 3), axis=0).max()
        tube = 0.003 * scene
        # the boundaries of the cells, on the surface, lifted by a tube radius
        lift = numpy.array([0.0, 0.0, -tube])
        knots = figures.Mesh()
        for k in range(0, grid.shape[0], per_cell):
            figures.add_tube(knots, p[k] + lift, tube)
        for k in range(0, grid.shape[1], per_cell):
            figures.add_tube(knots, p[:, k] + lift, tube)
        samples = figures.Mesh()
        for row in numpy.array(data['samples']):
            for q in row:
                figures.add_ball(samples, q, 2.8 * tube)
        name = 'terrain_' + interpolation
        model, m = terrain_model(work, interpolation)
        m['assemblies'][0]['bodies'][0]['geometry'] = [
            knots.write(work, name + '_knots', KNOT_COLOR, 1.0),
            samples.write(work, name + '_samples', NET_COLOR, 1.0)]
        with open(model, 'w') as file:
            json.dump(m, file)
        image = os.path.join(work, name + '.png')
        figures.SIZE = PANEL
        center, radius = bounds(p.reshape(-1, 3))
        figures.render(model, image, (-65, 22), center, 0.47 * radius)
        panels.append(image)
        if interpolation == 'bicubic':
            bicubic = data
        else:
            bilinear = data
    lettered(panels, work, 'terrain', os.path.join(directory, 'terrain.png'))
    print('terrain.png', flush=True)
    terrain_section(bicubic, bilinear, per_cell,
                    os.path.join(directory, 'terrain_section.png'))
    print('terrain_section.png', flush=True)


def terrain_section(bicubic, bilinear, per_cell, target):
    """height, slope and second derivative along the row of control points
    j = 2, with the control points of the row and the knots marked"""
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot
    pyplot.rcParams.update({
        'font.family': FONT, 'font.size': 11, 'mathtext.fontset': 'custom',
        'mathtext.rm': FONT, 'mathtext.it': FONT + ':italic'})
    row = 2
    figure, axes = pyplot.subplots(3, 1, figsize=(7.0, 6.2), sharex=True)
    for data, color, text in ((bicubic, U_COLOR, 'bicubic'),
                              (bilinear, V_COLOR, 'bilinear')):
        line = numpy.array(data['surface'])[row * per_cell]
        x, z, z_x, z_xx = line[:, 0], line[:, 2], line[:, 3], line[:, 5]
        at_knot = numpy.abs(x / data['x_scale'] -
                            numpy.round(x / data['x_scale'])) < 1e-9
        if not data['bicubic']:
            # zero inside the cells, undefined at the knots
            z_xx = numpy.where(at_knot, numpy.nan, z_xx)
            # the slope is drawn with a break at each knot, where it jumps
            z_x = numpy.where(at_knot, numpy.nan, z_x)
        axes[0].plot(x, -z, color=color, label=text)
        axes[1].plot(x, -z_x, color=color)
        axes[2].plot(x, -z_xx, color=color)
    control = numpy.array(bicubic['samples'])[row]
    axes[0].plot(control[:, 0], -control[:, 2], 'o', color='black',
                 markersize=4, label='samples')
    for axis in axes:
        for q in control[:, 0]:
            axis.axvline(q, color='0.8', linewidth=0.8, zorder=0)
    axes[0].set_ylabel('height (m)')
    axes[1].set_ylabel('slope')
    axes[2].set_ylabel('second derivative (1/m)')
    axes[2].set_xlabel('x (m)')
    axes[0].legend(frameon=False, ncol=3, loc='upper center',
                   bbox_to_anchor=(0.5, 1.25))
    figure.align_ylabels(axes)
    figure.tight_layout()
    figure.savefig(target, dpi=150)
    pyplot.close(figure)


# (x, y, yaw in degrees) of the bunnies of the broad phase figure: a group of
# three, a pair whose bounding spheres overlap, a pair whose spheres overlap
# only with their margins, and two alone
BUNNIES = [(0.0, 0.0, 0), (0.3, 0.05, 70), (0.12, 0.3, 160),
           (0.95, 0.0, 30), (1.27, 0.08, 200),
           (0.55, 0.75, 110), (1.0, 0.8, 300),
           (1.75, 0.55, 45), (-0.55, 0.75, 250)]
SPHERE_COLOR = (0.72, 0.48, 1.0)
MARGIN_COLOR = (0.6, 0.6, 0.6)
CANDIDATE_COLOR = (0.85, 0.06, 0.08)


def bunny_model(work, overlay):
    """the bunnies on float joints, each with a convex hull fitted to it, and
    the overlay meshes on the fixed body"""
    link = os.path.join(work, 'stanford_bunny.obj')
    if not os.path.exists(link):
        os.symlink(BUNNY, link)
        # light grey in place of the bunny's dark glaze, to be seen through
        # the spheres
        with open(os.path.join(work, 'stanford_bunny.mtl'), 'w') as file:
            file.write('newmtl bunny\nKd 0.7 0.7 0.7\nKs 0.2 0.2 0.2\n'
                       'Ns 30\n')
    model = figures.write_model(work, overlay, 'broadphase')
    with open(model) as file:
        m = json.load(file)
    assembly = m['assemblies'][0]
    for k, (x, y, yaw) in enumerate(BUNNIES):
        a = math.radians(yaw)
        turn = numpy.array([[math.cos(a), -math.sin(a), 0.0],
                            [math.sin(a), math.cos(a), 0.0],
                            [0.0, 0.0, 1.0]])
        name = 'bunny%d' % k
        assembly['bodies'].append({
            'name': name,
            'geometry': [{'type': 'mesh', 'filename': ['stanford_bunny.obj']}],
            'collision_geometry': [{'name': name, 'type': 'convex_hull'}],
            'center_of_mass': [0.0, 0.0, 0.0],
            'inertia': {'mass': 1.0, 'Ixx': 1.0, 'Iyy': 1.0, 'Izz': 1.0,
                        'Ixy': 0.0, 'Iyz': 0.0, 'Izx': 0.0}})
        assembly['joints'].append({
            'name': 'float%d' % k, 'type': 'float',
            'parent': [['fixed'], 'origin'], 'child': [[name], 'origin'],
            'initial_displacement': {'R': (turn @ UPRIGHT).tolist(),
                                     'T': [x, y, 0.0]},
            'initial_velocity': [0.0] * 6})
    with open(model, 'w') as file:
        json.dump(m, file)
    return model


def broadphase_figure(work, directory):
    model = bunny_model(work, [])
    data = os.path.join(work, 'broadphase.json')
    subprocess.run([TOOL, 'broadphase', model, data], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    with open(data) as file:
        phase = json.load(file)
    bounded = [k for k, g in enumerate(phase['geometry'])
               if g['radius'] is not None]
    spheres, margins, centers, links, bars = (figures.Mesh() for _ in range(5))
    for k in bounded:
        g = phase['geometry'][k]
        c = numpy.array(g['center'])
        smooth_ball(spheres, c, g['radius'])
        smooth_ball(margins, c, g['radius'] + g['margin'])
        figures.add_ball(centers, c, 0.018)
    # the candidates among the bunnies; each is also a candidate with the
    # floor, a plane, which is not drawn
    for a, b in phase['candidates']:
        if a in bounded and b in bounded:
            figures.add_tube(links, numpy.array(
                [phase['geometry'][a]['center'],
                 phase['geometry'][b]['center']]), 0.014)
    # the intervals of the enlarged spheres along the axis of the sweep, in
    # the order of their lower ends, between the bunnies and the camera
    axis = phase['sweep_axis']
    other = (axis + 1) % 3
    rows = sorted(bounded, key=lambda k: phase['geometry'][k]['center'][axis]
                  - phase['geometry'][k]['radius']
                  - phase['geometry'][k]['margin'])
    front = max(phase['geometry'][k]['center'][other] +
                phase['geometry'][k]['radius'] +
                phase['geometry'][k]['margin'] for k in bounded) + 0.12
    for row, k in enumerate(rows):
        g = phase['geometry'][k]
        reach = g['radius'] + g['margin']
        ends = []
        for end in (-reach, reach):
            p = numpy.zeros(3)
            p[axis] = g['center'][axis] + end
            p[other] = front + 0.07 * row
            ends.append(p)
        figures.add_tube(bars, numpy.array(ends), 0.016)
    overlay = [spheres.write(work, 'bounding', SPHERE_COLOR, 0.22),
               margins.write(work, 'margins', MARGIN_COLOR, 0.12),
               centers.write(work, 'centers', NET_COLOR, 1.0),
               links.write(work, 'candidates', CANDIDATE_COLOR, 1.0),
               bars.write(work, 'intervals', SPHERE_COLOR, 1.0)]
    model = bunny_model(work, overlay)
    points = [g['center'] for g in phase['geometry'] if g['radius']]
    center, radius = bounds(points + [[0.6, front + 0.07 * len(rows), 0.0]])
    image = os.path.join(work, 'broadphase.png')
    figures.SIZE = (1000, 640)
    # from +y, so that +x is to the right with +z down
    figures.render(model, image, (90, 62), center, 0.6 * radius)
    shutil.copy(image, os.path.join(directory, 'broadphase.png'))
    print('broadphase.png', flush=True)


def smooth_ball(mesh, center, radius, rows=32, columns=64):
    """a sphere of quads with the normals of the sphere"""
    for i in range(rows):
        for j in range(columns):
            corners, normals = [], []
            for a, b in ((i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)):
                t, p = math.pi * a / rows, 2.0 * math.pi * b / columns
                n = numpy.array([math.sin(t) * math.cos(p),
                                 math.sin(t) * math.sin(p), math.cos(t)])
                corners.append(center + radius * n)
                normals.append(n)
            mesh.add(corners, normals)


# the second link of a track on the pin of the first, as in the test
# CadContact.LinkHangsOnItsPin: its bushing sunk on the pin by the clearance
# (20 mm bore, 19 mm pin) and 20 um more
LINK_PITCH = 0.19
LINK_SINK = 0.001 + 20e-6
LINK_TURN = [[0, 0, 1], [1, 0, 0], [0, 1, 0]]
LINK_COLORS = [(0.62, 0.64, 0.68), (0.35, 0.52, 0.74)]


def link_model(work, overlay, opacity=(1.0, 1.0), count=2):
    """the two links, the second on the pin of the first, with the overlay
    meshes on the fixed body"""
    link = os.path.join(work, 'track_link.step')
    if not os.path.exists(link):
        os.symlink(os.path.join(CAD, 'track_link.step'), link)
    model = figures.write_model(work, overlay, 'links')
    with open(model) as file:
        m = json.load(file)
    assembly = m['assemblies'][0]
    for k in range(count):
        name = 'link%d' % (k + 1)
        assembly['bodies'].append({
            'name': name,
            'geometry': [{'type': 'cad', 'filename': 'track_link.step',
                          'color': list(LINK_COLORS[k]),
                          'opacity': opacity[k]}],
            'collision_geometry': [{'name': name, 'type': 'cad'}],
            'center_of_mass': [0.0974, 0.0, 0.0359],
            'inertia': {'mass': 38.06, 'Ixx': 0.75, 'Iyy': 0.15, 'Izz': 0.8,
                        'Ixy': 0.0, 'Iyz': 0.0, 'Izx': 0.0}})
        assembly['joints'].append({
            'name': name + '_joint', 'type': 'rigid' if k == 0 else 'float',
            'parent': [['fixed'], 'origin'], 'child': [[name], 'origin'],
            'initial_displacement': {
                'R': LINK_TURN,
                'T': [0.0, 0.0, 0.0] if k == 0 else [-LINK_SINK, LINK_PITCH,
                                                     0.0]}})
        if k == 1:
            assembly['joints'][-1]['initial_velocity'] = [0.0] * 6
    with open(model, 'w') as file:
        json.dump(m, file)
    return model


def cad_pair_data(work):
    model = link_model(work, [])
    data = os.path.join(work, 'cadpair.json')
    subprocess.run([TOOL, 'cadpair', model, data], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    with open(data) as file:
        return json.load(file)


BOX_COLOR = (0.95, 0.55, 0.10)
# neighbouring boxes of one level of the tree in turn
LEVEL_COLORS = [(0.95, 0.55, 0.10), (0.10, 0.45, 0.90), (0.15, 0.70, 0.25),
                (0.85, 0.15, 0.55)]
TALL = (400, 560)


def to_world(side, points):
    R = numpy.array(side['pose']['R'])
    return numpy.array(points) @ R.T + numpy.array(side['pose']['t'])


def add_box(mesh, side, box, radius):
    """the twelve edges of a box of a side's frame, in world coordinates"""
    lo, hi = numpy.array(box[0]), numpy.array(box[1])
    corner = lambda k: numpy.where([(k >> a) & 1 for a in range(3)], hi, lo)
    for k in range(8):
        for a in range(3):
            if not (k >> a) & 1:
                ends = to_world(side, [corner(k), corner(k | (1 << a))])
                figures.add_tube(mesh, ends, radius)


def boxtree_figure(work, directory, data):
    side = data['a']
    every = to_world(side, [p for f in side['faces'] for loop in f['loops']
                            for p in loop])
    center, scene = bounds(every)
    tube = 0.004 * scene
    panels = []
    deepest = max(n['depth'] for n in side['nodes'])
    # the boxes of the first level coincide with the root: each half of the
    # faces holds a plate that spans the length of the link
    for level in (0, 2, 3, deepest):
        boxes = [figures.Mesh() for _ in LEVEL_COLORS]
        nodes = [n for n in side['nodes'] if n['depth'] == level]
        for i, node in enumerate(nodes):
            add_box(boxes[i % len(boxes)], side, node['box'], tube)
        name = 'boxtree_%d' % level
        geometry = [m.write(work, '%s_%d' % (name, i), LEVEL_COLORS[i], 1.0)
                    for i, m in enumerate(boxes) if m.faces]
        model = link_model(work, geometry, count=1)
        image = os.path.join(work, name + '.png')
        figures.SIZE = TALL
        figures.render(model, image, (35, 25), center, 0.55 * scene)
        panels.append(image)
    lettered(panels, work, 'boxtree', os.path.join(directory, 'boxtree.png'))
    print('boxtree.png (deepest level %d)' % deepest, flush=True)


def stepcontact_figure(work, directory, data):
    a, b = data['a'], data['b']
    every = numpy.concatenate([
        to_world(s, [p for f in s['faces'] for loop in f['loops'] for p in loop])
        for s in (a, b)])
    center, scene = bounds(every)
    tube = 0.003 * scene
    panels = []
    # (a) the root boxes of the two links
    roots = [figures.Mesh(), figures.Mesh()]
    for k, s in enumerate((a, b)):
        add_box(roots[k], s, s['nodes'][0]['box'], tube)
    geometry = [roots[0].write(work, 'root_a', BOX_COLOR, 1.0),
                roots[1].write(work, 'root_b', LINK_COLORS[1], 1.0)]
    panels.append(step_render(work, 'step_roots', geometry, (0.55, 0.35),
                              center, scene))
    # (b) the boxes of the faces of each link that are in a candidate pair
    faces_a = sorted({fa for fa, fb in data['candidates']})
    faces_b = sorted({fb for fa, fb in data['candidates']})
    boxes = [figures.Mesh(), figures.Mesh()]
    for fa in faces_a:
        add_box(boxes[0], a, a['faces'][fa]['box'], 0.6 * tube)
    for fb in faces_b:
        # the box of b's face in a's frame, as it is tested against a's tree
        add_box(boxes[1], a, data['b_boxes_in_a'][fb], 0.6 * tube)
    geometry = [boxes[0].write(work, 'boxes_a', BOX_COLOR, 1.0),
                boxes[1].write(work, 'boxes_b', LINK_COLORS[1], 1.0)]
    panels.append(step_render(work, 'step_boxes', geometry, (0.55, 0.35),
                              center, scene))
    # (c) the outlines of the faces in candidate pairs
    outlines = [figures.Mesh(), figures.Mesh()]
    for k, (s, faces) in enumerate(((a, faces_a), (b, faces_b))):
        for f in faces:
            for loop in s['faces'][f]['loops']:
                figures.add_tube(outlines[k], to_world(s, loop), 0.6 * tube)
    geometry = [outlines[0].write(work, 'outlines_a', BOX_COLOR, 1.0),
                outlines[1].write(work, 'outlines_b', (0.10, 0.25, 0.85), 1.0)]
    # the faces and points of contact lie about the pin
    pin = numpy.array([c['point_a'] for c in data['contacts']])
    near, reach = bounds(pin)
    reach = max(1.8 * reach, 0.32 * scene)
    panels.append(step_render(work, 'step_faces', geometry, (0.35, 0.2),
                              near, reach))
    # (d) the points of contact and their normals
    points, arrows = figures.Mesh(), figures.Mesh()
    for c in data['contacts']:
        p = numpy.array(c['point_a'])
        figures.add_ball(points, p, 0.015 * reach)
        figures.add_arrow(arrows, p, c['normal'], 0.15 * reach, 0.006 * reach)
    geometry = [points.write(work, 'points', figures.COMPUTED, 1.0),
                arrows.write(work, 'normals', figures.COMPUTED, 1.0)]
    panels.append(step_render(work, 'step_points', geometry, (0.35, 0.2),
                              near, reach))
    lettered(panels, work, 'stepcontact',
             os.path.join(directory, 'stepcontact.png'))
    print('stepcontact.png', flush=True)


def step_render(work, name, geometry, opacity, center, scene):
    model = link_model(work, geometry, opacity=opacity)
    image = os.path.join(work, name + '.png')
    figures.SIZE = TALL
    figures.render(model, image, (35, 25), center, 0.75 * scene)
    return image


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        ROOT, 'doc', 'sphinx', 'contact', 'figures')
    os.makedirs(directory, exist_ok=True)
    only = sys.argv[2:]
    with tempfile.TemporaryDirectory() as work:
        if not only or 'shapes' in only:
            shapes_figure(work, directory)
        if not only or 'meshes' in only:
            meshes_figure(work, directory)
        if not only or 'broadphase' in only:
            broadphase_figure(work, directory)
        if not only or 'boxtree' in only or 'stepcontact' in only:
            data = cad_pair_data(work)
            if not only or 'boxtree' in only:
                boxtree_figure(work, directory, data)
            if not only or 'stepcontact' in only:
                stepcontact_figure(work, directory, data)
        if not only or 'terrain' in only:
            terrain_figure(work, directory)
        if not only or 'topology' in only:
            topology_figure(work, directory)
        for entry in SURFACES:
            if not only or entry[0] in only:
                surface_figure(work, directory, entry)


if __name__ == '__main__':
    main()
