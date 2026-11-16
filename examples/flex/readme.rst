Slider-crank with a flexible connecting rod
===========================================

``slider_crank_flexible_rod.json`` is a slider-crank in the horizontal x-y
plane whose steel connecting rod is a flexible body (``"type": "modal"``).
The crank starts at 150 rad/s and turns freely; the inertia of the rod and
of the slider bends the rod in the plane of the mechanism.

The rod is made from its STEP solid when the model loads: it is meshed with
10-node tetrahedra, each bore is tied to the revolute joint through it, and
the mesh is reduced to the six static modes of the small end and the four
lowest fixed-interface modes (about 885, 987, 2410 and 2680 Hz). The analysis
is kept in ``slider_crank_flexible_rod.slider_crank__connecting_rod.fe.h5``
next to the model, so it runs again only when the rod changes.

In the viewer, View > Deformation scale magnifies the bending (try 300), and
the rod's body panel opens its mode window. To render an image::

  build/shift/main/shift examples/flex/slider_crank_flexible_rod.json \
    --image rod.png --time 0.05 --deformation-scale 300 --view 270,60

``scripts/make_flex_step.py`` writes the STEP files of the rod, the crank
and the slider (in millimeters).
