Slider-crank with a flexible connecting rod
===========================================

``slider_crank_flexible_rod.json`` is a slider-crank in the horizontal x-y
plane whose steel connecting rod is a flexible body (``"type": "modal"``).
The crank starts at 20 rad/s and turns freely; the inertia of the rod and
of the slider bends the rod in the plane of the mechanism.

The rod is made from its STEP solid when the model loads: it is meshed with
10-node tetrahedra, each bore is tied to the revolute joint through it, and
the mesh is reduced to the six static modes of the small end and the four
lowest fixed-interface modes (about 885, 987, 2410 and 2680 Hz). The analysis
is kept in ``slider_crank_flexible_rod.slider_crank__connecting_rod.fe.h5``
next to the model, so it runs again only when the rod changes.

The slider is a piston: a cylindrical crown along the slide, clear of the
rod's small end, which turns on the wrist pin between two pin bosses.

In the viewer, View > Deformation scale magnifies the bending (try 15000;
the rod's slight stretch is magnified too, so at such a scale its small end
is drawn off the wrist pin);
View > Flexible bodies draws the surface of the mesh in place of the CAD, or
the edges of its elements over it, the nodes tied to the joints in its
bores as points, or its stress (the quantity, color map and range of Edit >
Settings: von Mises, principal, a component, or the safety factor against
the rod's ``yield_strength`` of 250 MPa, now or the greatest so far), for
every flexible body (the wrist pin hides the small
end's; hide the slider to see them), and the rod's body panel does so for
the rod alone and opens its mode window. To render an
image::

  build/shift/main/shift examples/flex/slider_crank_flexible_rod.json \
    --image rod.png --time 0.1 --deformation-scale 15000 --view 270,60 \
    --mesh-edges

``scripts/make_flex_step.py`` writes the STEP files of the rod, the crank
and the slider (in millimeters).
