#!/usr/bin/env python3
"""Write examples/terramechanics/track_frame_scm_fine.json, the track of
track_frame_scm.json on a finer grid of deformable sand, where the grousers
leave their marks in the rut.

It is made from track_frame_scm.json, so it keeps everything that model has
(its disabled_pairs too), and changes three things:

- the soil_contact geometry of each shoe: instead of one box down to the
  plane of the grouser tips, a box of the plate and one of each of the three
  grousers (the cross section of scripts/make_track_step.py's shoe_profile),
  so the soil is pressed deeper under the grousers than between them;
- the spacing of the grid of the terrain: GRID_SPACING instead of 0.05 m,
  so that the 20 mm grousers, 75 mm apart, cover two nodes each and the soil
  between them a few. The grid covers the same area as in
  track_frame_scm.json; only the nodes that change are stored, so the
  ground around the track costs little. The elevation map stays the same
  (track_sand.tiff);
- bulldozing: the sand the shoes press down is pushed to the rims of their
  prints and eroded to the angle of repose, so it rises between the
  grousers and along the rut.

A benchmark: it takes several times as long as track_frame_scm.json.

usage: make_track_scm_fine.py
"""

import json
import os
import sys

here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, here)
import make_track_step as track  # noqa: E402

GRID_SPACING = 0.01            # m

examples = os.path.join(here, os.pardir, 'examples', 'terramechanics')
with open(os.path.join(examples, 'track_frame_scm.json')) as f:
    model = json.load(f)

model['comment'] = model['comment'] + (
    ' This is the fine variant (scripts/make_track_scm_fine.py): the soil is '
    'pressed by the plate and the three grousers of each shoe, on a grid of '
    '%g mm, so the grousers leave their marks in the rut, and bulldozed: '
    'the sand pushed aside rises around the prints.'
    % (1000.0 * GRID_SPACING))


def mm(v):
    return round(v / 1000.0, 6)


def shoe_boxes():
    """the plate and the grousers, in the link's frame (x along the track
    from the bushing, z out of the track loop), as shoe_profile draws them"""
    x0, x1 = track.SHOE_GAP, track.PITCH - track.SHOE_GAP
    z0, z1 = track.SHOE_TOP, track.SHOE_TOP + track.SHOE_THICKNESS
    zg = z1 + track.GROUSER_HEIGHT
    h = track.GROUSER_THICKNESS / 2.0
    centers = [x0 + h + 5.0, track.PITCH / 2.0, x1 - h - 5.0]
    boxes = [('shoe_plate', 0.5 * (x0 + x1), 0.5 * (x1 - x0),
              0.5 * (z0 + z1), 0.5 * (z1 - z0))]
    for k, c in enumerate(centers):
        boxes.append(('grouser%d' % (k + 1), c, h, 0.5 * (z1 + zg),
                      0.5 * (zg - z1)))
    return [{'name': name, 'type': 'box', 'contact': False,
             'half_extents': [mm(hx), mm(track.SHOE_HALF_WIDTH), mm(hz)],
             'pose': {'T': [mm(x), 0.0, mm(z)]}}
            for name, x, hx, z, hz in boxes]


assembly = model['assemblies'][0]
links = 0
for body in assembly['bodies']:
    geometry = body.get('collision_geometry', [])
    shoe = [g for g in geometry if g.get('name') == 'shoe']
    if shoe:
        geometry.remove(shoe[0])
        geometry.extend(shoe_boxes())
        links += 1
names = [g['name'] for g in shoe_boxes()]
for module in assembly['force_modules']:
    if module['type'] == 'soil_contact':
        module['geometry'] = names

terrain = model['terrain'][0]
terrain['spacing'] = GRID_SPACING
# the angle of repose: the friction angle of dry sand. Bulldozed every
# millisecond (the default interval), with 30 passes of erosion each time:
# the berms settle to the angle of repose, as with 3 passes after every step
terrain['bulldozing'] = {'enable': True, 'flow_factor': 1.0,
                         'erosion_iterations': 30, 'interval': 0.001}
# loose dry sand, which compacts under the shoes before it is pushed aside:
# void ratios of about 0.8 as it lies and 0.5 compacted, typical of a uniform
# sand, with grains of specific gravity 2.65, give unit weights of 14.4 and
# 17.3 kN/m^3, so a layer as deep as the track is wide loses about 17% of its
# volume before soil rises beside the track. The unit weight also sets the
# earth pressure on the grousers (track_soil)
terrain['soil']['unit_weight'] = 14400.0
terrain['soil']['max_unit_weight'] = 17300.0

path = os.path.join(examples, 'track_frame_scm_fine.json')
with open(path, 'w') as f:
    json.dump(model, f, indent=2)
    f.write('\n')
nodes = ((int((terrain['x'][1] - terrain['x'][0]) / GRID_SPACING + 1e-9) + 1) *
         (int((terrain['y'][1] - terrain['y'][0]) / GRID_SPACING + 1e-9) + 1))
print('wrote %s: %d shoes of %d boxes, %d nodes'
      % (os.path.normpath(path), links, len(names), nodes))
