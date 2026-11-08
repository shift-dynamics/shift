#!/usr/bin/env python3
"""Write test/contact/README.md from readme_data.py, and check it against the
tests: every test of the files here is listed once, every figure exists and
every image is used.

  python3 test/contact/make_readme.py

Run it from the root of the repository, after a release build (it lists the
tests of build-release/test/shift-test, or of SHIFT_BUILD). The figures are made
by make_figures.py.
"""

import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from readme_data import FILTER, NO_FIGURE, SECTIONS  # noqa: E402

BUILD = os.environ.get('SHIFT_BUILD', 'build-release')

out = ['''# Contact tests

The tests of contact detection and contact forces. They are part of `shift-test`:

```bash
meson test -C build                                   # everything
build/test/shift-test --gtest_filter='CadRound.*'     # one group
```

| File | What it tests |
|------|---------------|
| `test_contact_geometry.cc` | Depth, normal and points of convex shapes and planes |
| `test_contact_primitives.cc` | Cylinders, capsules and boxes: fits, support points, line contact on a plane or a box |
| `test_contact_force.cc` | `force_module::Contact` in simulation, combining two materials, and the collision examples |
| `test_stick_slip.cc` | Stick-slip friction, the bristle model of `force_module::Contact` |
| `test_terrain_contact.cc` | `force_module::TerrainContact` on elevation maps |
| `test_cad_round_contact.cc` | Other shapes on the exact surfaces of STEP parts |
| `test_cad_contact.cc` | The sprocket, track links and idlers, and Hertz contact of an ellipse |
| `test_cad_line_contact.cc` | Lines on curved faces: cams, gear teeth, a bent ring |
| `test_collision_geometry_input.cc` | Collision geometry from the model input |

## Kinds of test

- **Closed form**: a pose whose contact can be worked out by hand. The test compares
  depth, normal and points with it.
- **Against another method**: the same contact found another way, such as a brute force
  search, a numerical integration, or the same shape from a STEP file.
- **No jump**: a pose swept in small steps, with the force required to change smoothly.
  A jump in a force stalls a variable step integrator.
- **Apart**: a pose just out of contact, where nothing may be found.
- **Settling**: a body simulated to rest, at the depth that carries its weight.
- **Input**: a model that must load as written, or be refused.

## Reading the figures

Each figure is ray traced by the viewer from what the test computed, with the bodies
transparent. The left panel shows the whole scene and the right panel a close up. Red is
what was computed:

- a **ball** for a point of contact
- a **tube** for a line of contact, with a ball at each of its stations
- a **thin plate** for points that share a normal and so bound a surface, with a ball at
  each point
- an **arrow** for the normal

The caption gives the number of lines, surfaces and points, the length of the lines and
the deepest point. A test with no contact, or one that only fits a shape, has one panel
with solid bodies, as has a test that ends with a body in the air after a rebound. A test keeps its contact with the most points, the last of those, so
a random test shows a late trial. A test run with several parameters shows its first.

''']
listed=[]
for title,intro,tests in SECTIONS:
    out.append('## %s\n\n%s\n\n'%(title,intro))
    for name,_,text in tests:
        listed.append(name)
        shown=name.replace('_','/',1) if '_' in name.split('.')[0] else name
        out.append('### `%s`\n\n%s\n\n![%s](images/%s.jpg)\n\n'%(shown,text,name.split('.')[1],name))
out.append('## Tests with no figure\n\nThese check input or arithmetic, so there is no contact to draw.\n\n')
out.append('| Test | What it checks |\n|------|----------------|\n')
for name,text in NO_FIGURE:
    listed.append(name)
    out.append('| `%s` | %s |\n'%(name.replace('_','/',1) if '_' in name.split('.')[0] else name,text))
out.append('''
## Making the figures

`contact_record.cc` writes what each test computed when `SHIFT_CONTACT_RECORD` names a
directory, and `make_figures.py` turns those records into models and renders them with the
viewer. It needs a release build, a display and ImageMagick, and windows appear as it
renders. The records of the track models are over 1 GB each.

```bash
mkdir -p /tmp/contact
SHIFT_CONTACT_RECORD=/tmp/contact build-release/test/shift-test --gtest_filter='%s'
python3 test/contact/make_figures.py /tmp/contact test/contact/images
```

The text of this file is in `readme_data.py`. `make_readme.py` writes it, and checks that
every test here is listed and every figure exists:

```bash
python3 test/contact/make_readme.py
```
''' % FILTER)
open(os.path.join(HERE, 'README.md'), 'w').write(''.join(out))
lst=subprocess.run([os.path.join(BUILD, 'test', 'shift-test'), '--gtest_list_tests'],capture_output=True,text=True).stdout
want=set(); suite=None
src=''.join(open(os.path.join(HERE, f)).read() for f in os.listdir(HERE) if f.endswith('.cc'))
suites=set(re.findall(r'^TEST\w*\((\w+),',src,re.M))
for l in lst.split('\n'):
    if l and not l.startswith(' ') and l.split('#')[0].strip().endswith('.'): suite=l.split('#')[0].strip()[:-1]
    elif l.startswith('  ') and suite:
        if suite.split('/')[-1] in suites and 'DISABLED' not in suite:
            want.add(suite.replace('/','_')+'.'+l.split('#')[0].strip().split('/')[0])
have=set(listed)
print('tests',len(want),'listed',len(have))
print('missing',sorted(want-have)); print('extra',sorted(have-want))
figs=[n for s in SECTIONS for n,_,_ in s[2]]
print('no image',len([n for n in figs if not os.path.exists(os.path.join(HERE, 'images', '%s.jpg' % n))]))
print('unused image',[f for f in os.listdir(os.path.join(HERE, 'images')) if f[:-4] not in figs])
