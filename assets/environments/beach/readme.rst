Beach
-----

A sand beach on the sea, an environment to try robots and vehicles in.
``examples/beach`` puts the Strandbeest on it.

``make_beach.py`` writes the beach: an elevation map of 120 m along the shore by 100 m across it, with dunes behind, a dry upper beach, a wet foreshore sloping into the sea and the sea floor; and a material map that lays out the surface materials of the model file, dry sand, damp sand, wet sand, grass on the dunes and the wet sand of the strip (below), kept apart so that no rocks lie on it. The three sands are one scan, the damp and the wet darker, more saturated and glossier, as water in the pores of sand makes it. Each material has its own look, its label for cameras and its soil. Boulders, rocks and stones from `../../scenery <../../scenery>`_ lie about the beach, the dunes and the surf, placed by the model's ``scenery``.

The beach is two terrains on one frame. The basic terrain ``beach`` is the scenery. The SCM terrain ``strip``, 13 m by 4 m of wet sand, is deformable: feet and wheels sink into it and leave their prints. Under the strip the scenery is lowered a little, so that the strip's surface is drawn over it.

The environment gives the sea, as water at height 0 with a low swell 60 m long rolling in (a period of 6.2 s) under the chop of a 5 m/s onshore breeze, breaking in the shallows and running up the sand, and a physical sky with the afternoon sun over the sea, whose air hazes the distance. None of it acts on the dynamics.

A model stands on the beach with the root keys that ``beach_parts(model_dir)`` of ``make_beach.py`` returns, its frame, terrains, surface materials, scenery and environment, their files named from ``model_dir``, the directory of the model file.

Regenerate the maps of the beach with::

  python3 make_beach.py

and then the models that stand on it, such as ``examples/beach/make_strandbeest_beach.py``.

Credit
~~~~~~

The sand, dry, damp and wet, is Poly Haven's Sand 01 by Rob Tuytel (https://polyhaven.com,
CC0 1.0 Universal, no credit required; given here as thanks), in
``textures/``.
