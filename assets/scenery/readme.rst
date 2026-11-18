Scenery
-------

Trees, saplings, shrubs, ferns, grass and rocks to strew over a terrain with
the ``scenery`` key of a model file (see the model file documentation), as
in `../environments/forest <../environments/forest>`_ and
`../environments/beach <../environments/beach>`_. Each mesh stands
with z down and its origin where it meets the ground, in meters.

* ``fir_a``, ``fir_b``, ``fir_c``, ``pine_a``, ``pine_b``, ``pine_c``:
  firs and pines 14.5 to 20.4 m tall, about 60,000 triangles each;
* ``fir_sapling_a`` to ``_c``, ``pine_sapling_a`` to ``_c``: young trees;
* ``shrub_02_a`` to ``_d``, ``fern_a``, ``fern_b``: undergrowth;
* ``grass_a`` to ``grass_e``: tufts of grass;
* ``mossy_rock_a`` to ``_f``, ``boulder``, ``stump``, ``dead_trunk``: rocks
  and dead wood;
* ``boulder_03``, ``boulder_04`` (2.5 to 3 m across), ``rock_07``,
  ``rock_09``, ``stone_01`` (15 to 30 cm), ``stones_a`` to ``_e`` (6 to
  15 cm): bare rocks and stones, as on the beach of
  `../environments/beach <../environments/beach>`_.

The leaves, needles and blades are textured cards, cut out by the alpha of
their textures (glTF's ``"alphaMode": "MASK"``). The textures are in
``textures/``, shared by the meshes that use them.

Credit
------

The meshes are made from the CC0 (public domain) models of Poly Haven
(https://polyhaven.com), at their 1k textures: their parts split into
separate variants, the cards of the leaves thinned to fewer, larger ones and
the solid parts decimated, so that a forest of them can be drawn; their
coordinates turned to z down.

* Fir Tree 01, Pine Tree 01, Fir Sapling, Pine Sapling Small, Fern 02,
  Grass Medium 01: Rob Tuytel and Rico Cilliers
* Shrub 02, Boulder 01: Rico Cilliers
* Tree Stump 01, Dead Tree Trunk: Rob Tuytel
* Rock Moss Set 01: Kless Gyzen
* Rock 07, Rock 09, Namaqualand Boulder 04: Jenelle van Heerden
* Namaqualand Boulder 03: Jenelle van Heerden and Dario Barresi
* Stone 01: Dario Barresi and Rico Cilliers
* Namaqualand Stones 01: Greg Zaal and Jenelle van Heerden

CC0 1.0 Universal (https://creativecommons.org/publicdomain/zero/1.0/): no
credit is required; it is given here as thanks.
