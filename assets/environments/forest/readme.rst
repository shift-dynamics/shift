Forest stage
------------

A gravel rally stage through a conifer forest, an environment to try
vehicles and their cameras in. ``examples/forest`` drives the Polo R WRC
along it.

``make_forest.py`` writes the stage: an elevation map 400 m by 300 m of
wooded hills, with a gravel road 850 m long cut into them, with a crown,
shoulders, ditches and the bumps of a gravel road, its grade smoothed along
it and the banks cut or filled to the hills on either side; and a material
map that lays out the surface materials of the model file: the forest floor
of needles and moss, the loose gravel of the road, the packed wheel lines,
the grass of the verges and bare rock on the banks of the deeper cuttings.
Each material has its own look, its label for cameras and its soil.

Firs and pines, saplings, shrubs and ferns, grass and rocks from
`../../scenery <../../scenery>`_ are strewn over the ground (the ``scenery`` of the
model file): the trees on the forest floor and back from the verges, the
grass thickest on the verges. The environment gives a physical sky with a
late morning sun, and a light haze among the trees.

The road runs from the start, at the origin heading +x, through a fast
right, a hairpin left, a long downhill right and an S to the finish. Two
crests make it a rally stage: a jump on the straight after the start, 1 m
high, met at about 20 m/s, where a car leaves the road for about half a
second and lands on its long far side; and a smaller crest at the turn in
of the long left at the top of the climb, which tightens to a radius of
37 m and falls to the outside (off camber), where a car braking into it
comes down light and slides; the gravel there is widened by 5 m on the
outside, a run-off that keeps a car sliding wide on it. Their places and
sizes are ``JUMPS``, ``CAMBERS`` and ``RUN_OFFS`` in ``make_forest.py``.

A model drives the stage with the root keys that ``forest_parts(model_dir)``
of ``make_forest.py`` returns, its frame, terrain, surface materials,
environment and scenery, their files named from ``model_dir``, the
directory of the model file; and ``driver()``, a driver controller along
the middle of the road, among its controllers.

The textures of the ground are from Poly Haven (https://polyhaven.com),
CC0: Rocky Trail and Sparse Grass by Amal Kumar, Forest Leaves 02 and Mossy
Rock by Rob Tuytel.

Regenerate the maps of the stage with::

  python3 make_forest.py

and then the models that drive it, such as
``examples/forest/make_polo_r_wrc_forest.py``.
