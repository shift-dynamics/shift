Polo R WRC on the forest stage
------------------------------

``polo_r_wrc_forest.json`` is the Polo R WRC of
`../polo_r_wrc <../polo_r_wrc>`_ driven along the gravel rally stage of
`../../assets/environments/forest <../../assets/environments/forest>`_.

A driver controller (``"type": "driver"``) follows the middle of the road: it
steers by pure pursuit, and sets the throttle, the brakes and the gears for
the speed that the bends allow, through maps to the car's steering rack,
engine, brakes and gearbox. It takes the bends at up to 4.5 m/s^2. The car
leaves the road for half a second over the jump after the start, at 19 m/s
(about 4 s in), and slides through the off-camber left over a crest at the
top of the climb, at up to 11 degrees between its heading and its path,
wide onto the run-off (about 41 s in); it finishes in about 59 s. The clutch is let in at the start as in the Polo's
own example.

Run it from the repository's root with the viewer::

  build-release/shift/main/shift examples/forest/polo_r_wrc_forest.json

``make_polo_r_wrc_forest.py`` writes it, from ``../polo_r_wrc/polo_r_wrc.json``
and ``forest_parts()`` and ``driver()`` of the stage's ``make_forest.py``.
Regenerate it from this directory, after
``../polo_r_wrc/generate_polo_r_wrc.py``::

  python3 make_polo_r_wrc_forest.py
