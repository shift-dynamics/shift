.. title:: Track

Track
=====

``"type": "track"`` measures a track of links, each on its own joints, and
applies no force. The track's contact forces come from the contact module
(:doc:`contact`) between the exact STEP surfaces of the links, the sprocket,
the idler and the rollers (see ``scripts/make_track_step.py`` for the parts of
the examples), and the wear of each surface from its ``wear_coefficient``.

The outputs are measured in the coordinates of the track frame, the body that
carries the wheels and rollers (``frame``, the fixed body by default), with
the directions ``up`` (default [0, 0, -1], z down) and ``lateral`` (along the
axles, default [0, 1, 0]):

* ``<span>_sag``, for each span between two supports (the wheels or rollers at
  its ends): how far the bushings of the links between them hang below the line
  through the tops of the supports, each support's origin plus its radius along
  ``up``. The radius of a wheel is that of the circle of the bushings around it;
  of a roller, the distance from its axle to the bushings of the links on it.
  Links whose bushings are above the line count as 0, and links below the line
  through the supports' origins (the other run of the track) are left out.
* ``misguide``: the greatest distance of a link from the middle of the track,
  along ``lateral``, from ``center`` (default 0).
* ``side_force``: the sum of the sizes of the lateral parts of the contact
  forces between the links and the ``guides``, the rollers whose flanges guide
  the links, as of the last evaluation of the forces.

.. code:: javascript

   {"type": "track", "name": "track", "frame": "frame",
    "links": ["link1", "link2"], "guides": ["roller1", "carrier1"],
    "up": [0, 0, -1], "lateral": [0, 1, 0],
    "spans": [{"name": "front", "supports": ["idler", "carrier1"],
               "radii": [0.322, 0.1205]}]}

Keys of ``track``:

.. shift-parameters:: shift/core/force_module/track.cc
   :function: force_module::Track::DeclareParameters
   :exclude: name, type, active

Each entry of ``spans``:

.. shift-parameters:: shift/core/force_module/track.cc
   :function: force_module::TrackSpan::DeclareParameters

Example
-------

* :source:`examples/cad/track_frame.json`: the track of 44 links on the track
  frame of half of a 20 t excavator, with five track rollers and a carrier
  roller (a benchmark: it is not run in the regular tests).
* Tests: :source:`test/test_cad_contact.cc`.
