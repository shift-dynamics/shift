#!/usr/bin/env python3
"""Render the figures of test/contact/README.md with the viewer, from what the
tests computed.

  mkdir -p /tmp/contact
  SHIFT_CONTACT_RECORD=/tmp/contact build-release/test/shift-test \\
      --gtest_filter='<the suites of the README>'
  python3 test/contact/make_figures.py /tmp/contact test/contact/images

Each test leaves a record of its geometry and of the contact points it computed
(see contact_record.h). This writes each record as a model of shift, its surfaces
and what was computed as meshes, and ray traces it twice with the viewer: the whole
scene on the left and the contact close up on the right, the bodies transparent so
that it can be seen between them. Red is what was computed: a ball for a point, a tube along a line of contact with a white
ball at each station, a thin plate for points that share a normal and so bound a
surface, and an arrow for the normal. Of the variants of a parameterized test only
the first is drawn, except for the tests of GRIDS, whose variants are different
shapes: each is drawn, in a grid with a label. The viewer needs a display, and shows a window as it renders.
SHIFT_BUILD names the build directory (default: build-release).
"""

import glob
import json
import math
import os
import re
import subprocess
import sys
import tempfile

import numpy

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
VIEWER = os.path.join(os.environ.get('SHIFT_BUILD', os.path.join(ROOT, 'build-release')),
                      'shift', 'main', 'shift')
COLORS = [(0.35, 0.52, 0.74), (0.85, 0.66, 0.36), (0.50, 0.68, 0.48),
          (0.68, 0.52, 0.72)]
PLANE_COLOR = (0.72, 0.72, 0.68)
COMPUTED = (0.85, 0.06, 0.08)
OPACITY = 0.28
# the font of the text of the documentation (the alabaster theme's font_family)
FONT = 'Georgia'
MOST_QUADS = 800000
SIZE = (480, 400)
SAMPLES = 48


def unit(v):
    length = numpy.linalg.norm(v)
    return v / length if length > 0.0 else v


def clusters(contact):
    """the lines (runs of points with a length), and the others grouped by
    normal into patches of three or more, or single points. Points with no
    pair, as a terrain's, are single"""
    points = list(contact['points'])
    if not contact['a']:
        return [], [], points
    lines, single = [], []
    run = []
    for p in points:
        if p['length'] > 0.0:
            run.append(p)
            if len(run) == 3:
                lines.append(run)
                run = []
        else:
            single.append(p)
    if run:
        lines.append(run)
    patches, loose = [], []
    while single:
        first = single.pop(0)
        group = [first]
        n = numpy.array(first['normal'])
        rest = []
        for p in single:
            (group if abs(numpy.dot(n, p['normal'])) > 0.99 else rest).append(p)
        single = rest
        if len(group) >= 3 and spread(group) > 1e-9:
            patches.append(group)
        else:
            loose += group
    return lines, patches, loose


def spread(group):
    """the second largest extent of points: zero if they are in a line"""
    p = numpy.array([g['a'] for g in group])
    return numpy.linalg.svd(p - p.mean(axis=0), compute_uv=False)[1]


def line_path(run, axis):
    """a line of contact from end to end: its three stations are at 0.775 of
    the way from its middle to its ends, and a parabola through them bends with
    the line. A shape on a plane has a single point in the middle of its line,
    which runs along the shape's axis."""
    p = numpy.array([r['a'] for r in run])
    if len(p) == 1 and axis is not None:
        n = unit(numpy.array(run[0]['normal']))
        d = unit(axis - numpy.dot(axis, n) * n)
        return numpy.array([p[0] - 0.5 * run[0]['length'] * d,
                            p[0] + 0.5 * run[0]['length'] * d])
    if len(p) != 3:
        return p
    t = numpy.array([-0.7746, 0.0, 0.7746])
    s = numpy.linspace(-1.0, 1.0, 25)
    return numpy.column_stack([numpy.polyval(numpy.polyfit(t, p[:, k], 2), s)
                               for k in range(3)])


def hull(points):
    """the order of points around their convex outline in their plane"""
    center = points.mean(axis=0)
    _, _, axes = numpy.linalg.svd(points - center)
    flat = (points - center) @ axes[:2].T
    order = numpy.argsort(numpy.arctan2(flat[:, 1], flat[:, 0]))
    return points[order]


def describe(record):
    """the numbers of what was computed, in words"""
    lines = patches = single = 0
    length, depth = 0.0, 0.0
    for contact in record['contacts']:
        ln, pa, lo = clusters(contact)
        lines += len(ln)
        patches += len(pa)
        single += len(lo)
        for p in contact['points']:
            length += p['length']
            depth = max(depth, p['depth'])
    words = []
    if lines:
        words.append('%d line%s, %.1f mm' % (lines, 's' * (lines > 1),
                                             length * 1e3))
    if patches:
        words.append('%d surface%s' % (patches, 's' * (patches > 1)))
    if single:
        words.append('%d point%s' % (single, 's' * (single > 1)))
    if not words:
        return 'no contact'
    if depth > 0.0:
        words.append('deepest %s' % (('%.3g um' % (depth * 1e6)) if depth < 1e-3
                                     else ('%.3g mm' % (depth * 1e3))))
    return ', '.join(words)


def size(metres):
    return '%.3g m' % metres if metres >= 1.0 else '%.3g mm' % (metres * 1e3)


class Mesh:
    """triangles and quads with normals, written as a Wavefront file"""

    def __init__(self):
        self.vertices, self.normals, self.faces = [], [], []

    def add(self, points, normals):
        start = len(self.vertices) + 1
        self.vertices += [tuple(p) for p in points]
        self.normals += [tuple(n) for n in normals]
        self.faces.append(range(start, start + len(points)))

    def add_quads(self, quads, normals):
        mean = normals.mean(axis=1)
        winding = numpy.cross(quads[:, 2] - quads[:, 0], quads[:, 3] - quads[:, 1])
        for q, n, w, m in zip(quads, normals, winding, mean):
            if numpy.dot(w, m) < 0.0:
                q, n = q[::-1], n[::-1]
            self.add(q, n)

    def write(self, directory, name, color, opacity):
        with open(os.path.join(directory, name + '.mtl'), 'w') as file:
            file.write('newmtl m\nKd %g %g %g\nKa 0 0 0\nKs 0.2 0.2 0.2\n'
                       'Ns 30\nd %g\n' % (*color, opacity))
        with open(os.path.join(directory, name + '.obj'), 'w') as file:
            file.write('mtllib %s.mtl\nusemtl m\n' % name)
            file.writelines('v %.9g %.9g %.9g\n' % v for v in self.vertices)
            file.writelines('vn %.6g %.6g %.6g\n' % n for n in self.normals)
            for face in self.faces:
                file.write('f %s\n' % ' '.join('%d//%d' % (i, i) for i in face))
        return {'type': 'mesh', 'filename': [name + '.obj', name + '.mtl']}


def frame_of(direction):
    d = unit(numpy.array(direction, dtype=float))
    u = unit(numpy.cross(d, [0.31, 0.57, 0.76]))
    return d, u, numpy.cross(d, u)


def add_tube(mesh, path, radius):
    ring = [(math.cos(a), math.sin(a))
            for a in numpy.linspace(0.0, 2.0 * math.pi, 13)]
    for p, q in zip(path[:-1], path[1:]):
        if numpy.linalg.norm(q - p) == 0.0:
            continue
        _, u, v = frame_of(q - p)
        for (c0, s0), (c1, s1) in zip(ring[:-1], ring[1:]):
            n0, n1 = c0 * u + s0 * v, c1 * u + s1 * v
            mesh.add([p + radius * n0, p + radius * n1, q + radius * n1,
                      q + radius * n0], [n0, n1, n1, n0])


def add_ball(mesh, center, radius):
    rows = numpy.linspace(0.0, math.pi, 11)
    columns = numpy.linspace(0.0, 2.0 * math.pi, 17)

    def n(t, p):
        return numpy.array([math.sin(t) * math.cos(p), math.sin(t) * math.sin(p),
                            math.cos(t)])
    for t0, t1 in zip(rows[:-1], rows[1:]):
        for p0, p1 in zip(columns[:-1], columns[1:]):
            corners = [n(t0, p0), n(t1, p0), n(t1, p1), n(t0, p1)]
            mesh.add([center + radius * c for c in corners], corners)


def add_arrow(mesh, start, direction, length, radius):
    d, u, v = frame_of(direction)
    tip = start + length * d
    neck = start + 0.72 * length * d
    add_tube(mesh, [start, neck], radius)
    around = numpy.linspace(0.0, 2.0 * math.pi, 17)
    for a0, a1 in zip(around[:-1], around[1:]):
        n0 = math.cos(a0) * u + math.sin(a0) * v
        n1 = math.cos(a1) * u + math.sin(a1) * v
        mesh.add([neck + 2.6 * radius * n0, neck + 2.6 * radius * n1, tip],
                 [n0, n1, d])


def add_plate(mesh, outline, normal, thickness):
    n = unit(numpy.array(normal, dtype=float))
    top = [p + 0.5 * thickness * n for p in outline]
    bottom = [p - 0.5 * thickness * n for p in outline]
    mesh.add(top, [n] * len(top))
    mesh.add(bottom[::-1], [-n] * len(top))
    for i in range(len(outline)):
        j = (i + 1) % len(outline)
        side = unit(numpy.cross(top[j] - top[i], n))
        mesh.add([top[i], bottom[i], bottom[j], top[j]], [side] * 4)


def long_axis(contact, geometry):
    """the longest direction of the more elongated solid of a pair (a cylinder
    or a capsule on a box, not the box)"""
    best, axis = 0.0, None
    for g in geometry:
        if 'quads' in g and g['name'] in (contact['a'], contact['b']):
            v = vertices_of(g)
            _, sizes, directions = numpy.linalg.svd(v - v.mean(axis=0),
                                                    full_matrices=False)
            elongation = sizes[0] / max(sizes[1], 1e-300)
            if elongation > best:
                best, axis = elongation, directions[0]
    return axis


def reach_of(contact, geometry):
    """the points of a pair, with the ends of its lines"""
    lines, patches, loose = clusters(contact)
    reach = [numpy.array(p['a']) for p in contact['points']]
    for run in lines:
        reach += list(line_path(run, long_axis(contact, geometry)))
    return reach


def contact_mesh(contacts, geometry, scale):
    """what was computed, drawn to be seen at a view of radius scale"""
    mesh = Mesh()
    tube, ball = 0.016 * scale, 0.03 * scale
    for contact in contacts:
        lines, patches, loose = clusters(contact)
        for run in lines:
            add_tube(mesh, line_path(run, long_axis(contact, geometry)), tube)
            for r in run:
                add_ball(mesh, numpy.array(r['a']), ball)
            middle = run[len(run) // 2]
            add_arrow(mesh, numpy.array(middle['a']), middle['normal'],
                      0.3 * scale, 0.5 * tube)
        for group in patches:
            add_plate(mesh, hull(numpy.array([g['a'] for g in group])),
                      group[0]['normal'], 0.012 * scale)
            for g in group:
                add_ball(mesh, numpy.array(g['a']), ball)
            add_arrow(mesh, numpy.array(group[0]['a']), group[0]['normal'],
                      0.3 * scale, 0.5 * tube)
        for g in loose:
            add_ball(mesh, numpy.array(g['a']), 1.3 * ball)
            add_arrow(mesh, numpy.array(g['a']), g['normal'], 0.3 * scale,
                      0.5 * tube)
    return mesh


def write_model(directory, geometry, name):
    model = {
        'model_parameters': {
            'version': '0.2',
            'units': {'angle': 'radians', 'length': 'meters',
                      'mass': 'kilograms', 'time': 'seconds'},
            'gravity': [0.0, 0.0, 0.0], 'solvers': [{'type': 'rnca'}],
            'integrators': [{'type': 'ode113', 'time_span': 0.0,
                             'time_step': 0.01}]},
        'assemblies': [{
            'name': 'scene',
            'bodies': [{'name': 'scene', 'geometry': geometry,
                        'center_of_mass': [0.0, 0.0, 0.0],
                        'inertia': {'mass': 1.0, 'Ixx': 1.0, 'Iyy': 1.0,
                                    'Izz': 1.0, 'Ixy': 0.0, 'Iyz': 0.0,
                                    'Izx': 0.0}}],
            'joints': [{'name': 'weld', 'type': 'rigid',
                        'parent': [['fixed'], 'origin'],
                        'child': [['scene'], 'origin']}]}]}
    path = os.path.join(directory, name + '.json')
    with open(path, 'w') as file:
        json.dump(model, file)
    return path


def angles_of(eye):
    return (math.degrees(math.atan2(eye[1], eye[0])),
            math.degrees(math.asin(eye[2])))


def contact_axes(points):
    """the mean normal, the direction the points run along and the third"""
    first = numpy.array(points[0]['normal'])
    n = unit(numpy.sum([numpy.array(p['normal']) *
                        (1.0 if numpy.dot(p['normal'], first) >= 0.0 else -1.0)
                        for p in points], axis=0))
    a = numpy.array([p['a'] for p in points])
    along = numpy.array([0.0, 0.0, 1.0])
    if len(a) > 1 and numpy.ptp(a, axis=0).max() > 1e-9:
        along = numpy.linalg.svd(a - a.mean(axis=0), full_matrices=False)[2][0]
    elif abs(numpy.dot(along, n)) > 0.9:
        along = numpy.array([0.0, 1.0, 0.0])
    along = unit(along - numpy.dot(along, n) * n)
    return n, along, numpy.cross(n, along)


def kept_body(record, geometry):
    """the body the close up keeps: the one in the most pairs, a plane first"""
    counts = {}
    for c in record['contacts']:
        if c['points']:
            for name in (c['a'], c['b']):
                counts[name] = counts.get(name, 0) + 1
    planes = {g['name'] for g in geometry if 'quads' not in g}
    return max(counts, key=lambda name: (counts[name], name in planes))


def vertices_of(g):
    return numpy.array(g['quads'], dtype=float).reshape(-1, 3)


def outward(g_other, at, n, reach):
    """the normal n, turned towards the part of the other body near at"""
    if g_other is None:
        return n
    if 'quads' not in g_other:
        return n if numpy.dot(at - numpy.array(g_other['plane_point']), n) >= 0.0 else -n
    v = vertices_of(g_other)
    near = v[numpy.linalg.norm(v - at, axis=1) < reach]
    mean = (near if len(near) else v).mean(axis=0)
    return n if numpy.dot(mean - at, n) >= 0.0 else -n


def render(model, image, angles, center, radius):
    command = [VIEWER, os.path.basename(model), '--image', image,
               '--size', '%dx%d' % SIZE, '--samples', str(SAMPLES), '--no-floor',
               '--view', '%g,%g' % angles,
               '--frame', '%.9g,%.9g,%.9g,%.9g' % (*center, radius)]
    subprocess.run(command, cwd=os.path.dirname(model), check=True, timeout=180,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def plane_mesh(g, at, radius, towards):
    n = unit(numpy.array(g['plane_normal']))
    p = numpy.array(g['plane_point'])
    at = at - numpy.dot(at - p, n) * n
    _, u, v = frame_of(n)
    up = n if numpy.dot(towards - at, n) >= 0.0 else -n
    mesh = Mesh()
    mesh.add([at - radius * u - radius * v, at + radius * u - radius * v,
              at + radius * u + radius * v, at - radius * u + radius * v], [up] * 4)
    return mesh


def draw(path, directory):
    with open(path) as file:
        record = json.load(file)
    # only the geometry in contact, if a model has more
    contacts = [c for c in record['contacts'] if c['points']]
    touching = {name for c in contacts for name in (c['a'], c['b']) if name}
    geometry = record['geometry']
    if touching:
        geometry = [g for g in geometry if g['name'] in touching]
    solid = [g for g in geometry if 'quads' in g]
    points = numpy.array([p['a'] for c in contacts for p in c['points']]
                         or [[0.0, 0.0, 0.0]])
    if solid:
        every = numpy.vstack([vertices_of(g) for g in solid])
        low, high = every.min(axis=0), every.max(axis=0)
    else:
        low, high = points.min(axis=0) - 0.05, points.max(axis=0) + 0.05
    if contacts:
        low = numpy.minimum(low, points.min(axis=0))
        high = numpy.maximum(high, points.max(axis=0))
    scene = max(0.5 * numpy.linalg.norm(high - low), 1e-6)
    center = 0.5 * (low + high)
    total = sum(len(g['quads']) for g in solid)
    stride = max(1, total // MOST_QUADS)
    with tempfile.TemporaryDirectory() as work:
        parts = {}
        for i, g in enumerate(geometry):
            if 'quads' in g:
                mesh = Mesh()
                mesh.add_quads(
                    numpy.array(g['quads'], dtype=float).reshape(-1, 4, 3)[::stride],
                    numpy.array(g['normals'], dtype=float).reshape(-1, 4, 3)[::stride])
                color = COLORS[i % len(COLORS)]
            else:
                at = points.mean(axis=0) if contacts else center
                towards = center if solid else at + numpy.array(g['plane_normal'])
                mesh = plane_mesh(g, at, 1.2 * scene, towards)
                color = PLANE_COLOR
            parts[g['name']] = mesh.write(work, 'part%d' % i, color,
                                          OPACITY if contacts else 1.0)
        images = []

        def shoot(label, names, shown, angles, middle, radius):
            drawn = [parts[n] for n in names]
            if shown:
                drawn.append(contact_mesh(shown, geometry, radius).write(
                    work, 'contact_' + label, COMPUTED, 1.0))
            image = os.path.join(work, label + '.png')
            render(write_model(work, drawn, label), image, angles, middle, radius)
            images.append(image)

        half = 0.0
        if not contacts:
            shoot('whole', list(parts), [], (35.0, 25.0), center, scene)
        else:
            every_point = [p for c in contacts for p in c['points']]
            n, along, across = contact_axes(every_point)
            mean = points.mean(axis=0)
            planes = [g for g in geometry if 'quads' not in g]
            up = outward(planes[0], center, n, scene) if planes else n
            shoot('whole', list(parts), contacts,
                  angles_of(unit(0.85 * across + 0.42 * along + 0.32 * up)),
                  center, scene)
            keep = kept_body(record, geometry)
            mine = [c for c in contacts if keep in (c['a'], c['b'])]
            mine_points = [p for c in mine for p in c['points']]
            # the point on the kept body's own surface
            for c in mine:
                if c['b'] == keep:
                    c['points'] = [dict(p, a=p['b']) for p in c['points']]
            at = numpy.array([q for c in mine for q in reach_of(c, geometry)])
            focus = 0.5 * (at.min(axis=0) + at.max(axis=0))
            half = max(1.25 * 0.5 * numpy.ptp(at, axis=0).max(), 0.08 * scene)
            n, along, across = contact_axes(mine_points)
            other_name = mine[0]['b'] if mine[0]['a'] == keep else mine[0]['a']
            other = next((g for g in geometry if g['name'] == other_name), None)
            face = outward(other, focus, n, 3.0 * half)
            shoot('close', list(parts), mine,
                  angles_of(unit(0.85 * face + 0.45 * across + 0.28 * along)),
                  focus, half)
        name = re.sub(r'_\d+$', '', os.path.splitext(os.path.basename(path))[0])
        target = os.path.join(directory, name + '.jpg')
        note = 'whole scene, %s across' % size(2.0 * scene)
        if contacts:
            note += '     |     close up, %s across' % size(2.0 * half)
        subprocess.run(
            ['magick', *images, '+append', '+repage', '-background', 'white',
             '-gravity', 'south', '-splice', '0x44', '-font', FONT,
             '-pointsize', '14', '-fill',
             '#d00f14', '-annotate', '+0+22', 'computed: ' + describe(record),
             '-fill', '#555555', '-pointsize', '12', '-annotate', '+0+5', note,
             '-quality', '86', target], check=True)
    return name


# tests whose variants are different shapes: every variant drawn, in a grid
# with a label each, from the variant's index
PAIR_TYPES = ['hull', 'ellipsoid', 'sphere', 'box', 'cylinder', 'capsule']
GRIDS = {
    'AllTypes_ContactGeometryPairs.MinimumTranslationSeparates':
        lambda k: '%s and %s' % (PAIR_TYPES[k // 6], PAIR_TYPES[k % 6]),
}


def draw_grid(name, paths, directory):
    """every variant of a test, its whole scene and close up, in a grid of
    six across, each labelled"""
    label = GRIDS[name]
    index = lambda p: int(re.search(r'_(\d+)\.json$', p).group(1))
    with tempfile.TemporaryDirectory() as work:
        tiles = []
        for path in sorted(paths, key=index):
            k = index(path)
            copy = os.path.join(work, '%s.json' % k)
            os.symlink(os.path.abspath(path), copy)
            draw(copy, work)
            with open(path) as file:
                computed = describe(json.load(file))
            tile = os.path.join(work, 'tile%02d.jpg' % k)
            # the panels without their caption, too small to read here, and
            # the pair and what was computed above them
            subprocess.run(
                ['magick', os.path.join(work, '%d.jpg' % k), '-gravity',
                 'south', '-chop', '0x44', '-resize', '480x', '-gravity',
                 'north', '-background', 'white', '-splice', '0x40',
                 '-font', FONT, '-pointsize', '15', '-fill', '#222222', '-annotate', '+0+3',
                 label(k), '-pointsize', '13', '-fill', '#d00f14',
                 '-annotate', '+0+21', computed, tile], check=True)
            tiles.append(tile)
        rows = []
        for r in range(0, len(tiles), 6):
            row = os.path.join(work, 'row%02d.jpg' % r)
            subprocess.run(['magick', *tiles[r:r + 6], '+append', row],
                           check=True)
            rows.append(row)
        subprocess.run(['magick', *rows, '-append', '-quality', '82',
                        os.path.join(directory, name + '.jpg')], check=True)
    return name


def first_variants(paths):
    """of the variants name_0, name_1 of a test, the first"""
    chosen = {}
    for path in sorted(paths):
        name = os.path.splitext(os.path.basename(path))[0]
        chosen.setdefault(re.sub(r'_\d+$', '', name), path)
    return list(chosen.values())


def main():
    records, directory = sys.argv[1], sys.argv[2]
    os.makedirs(directory, exist_ok=True)
    every = glob.glob(os.path.join(records, '*.json'))
    only = sys.argv[3:]
    if only:
        every = [p for p in every if any(o in p for o in only)]
    for name in GRIDS:
        variants = [p for p in every
                    if re.sub(r'_\d+$', '', os.path.splitext(
                        os.path.basename(p))[0]) == name]
        if variants:
            print(draw_grid(name, variants, directory), flush=True)
            every = [p for p in every if p not in variants]
    for path in first_variants(every):
        print(draw(path, directory), flush=True)


if __name__ == '__main__':
    main()
