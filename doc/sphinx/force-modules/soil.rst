.. _soil:
.. title:: Soil

Soil
====

Soft soil carries a wheel or a track shoe by its pressure, and pushes it along
by its shear. Two force modules use the same description of the soil: the
Bekker-Wong wheel (``"tire_type": "bekker_wong"`` of the :doc:`tire`) and the
track soil module (``"type": "track_soil"``, below).

Describing a soil
-----------------

A soil is given by name, from the table below, or by its properties, in SI
units and radians. Any property given beside a name replaces the table's
value:

.. code:: javascript

   "soil": {"name": "lete_sand", "shear_modulus": 0.011, "unit_weight": 15000}

   "soil": {"sinkage_exponent": 0.8, "cohesive_modulus": 1e5,
            "frictional_modulus": 5e6, "cohesion": 1300,
            "friction_angle": 0.54, "shear_modulus": 0.01}

.. shift-parameters:: shift/core/force_module/soil.cc
   :function: force_module::Soil::DeclareParameters

The pressure under a plate of width :math:`b` pressed in by :math:`z` is
Bekker's :math:`p = (k_c/b + k_\phi)\,z^n`, with the cohesive modulus
:math:`k_c`, the frictional modulus :math:`k_\phi` and the sinkage exponent
:math:`n`. Its shear strength is :math:`c + p\tan\phi` for the cohesion
:math:`c` and the friction angle :math:`\phi`, reached as the plate slides, by
Janosi and Hanamoto's :math:`1 - e^{-j/k_j}` after the slide :math:`j`
(:math:`k_j` is the shear modulus). The soil's unit weight gives the earth
pressure on the face of a grouser or the side of a shoe or wheel, from Rankine's
theory of passive pressure: on a vertical face down to the depth :math:`h` it
is :math:`\tfrac12 w h^2 \kappa_p + 2 c h \sqrt{\kappa_p}` per unit of width, with
:math:`\kappa_p = \tan^2(45^\circ + \phi/2)`.

The table
---------

The values of the first fourteen soils are those tabulated by Wong
:cite:`wong2001` and reproduced by :cite:`schepelmann2025`; the moisture
content is that of the sample. The last is the lunar soil simulant of
:cite:`kumar2019`. Wong's table gives no shear modulus or unit weight, so
those default to 0.01 m and 16 kN/m\ :sup:`3`: set them for the soil at hand.
The moduli are in kN/m\ :sup:`n+1` and kN/m\ :sup:`n+2`; the properties in the
JSON are in N and m.

.. list-table::
   :header-rows: 1

   * - name
     - moisture
     - :math:`n`
     - :math:`k_c`
     - :math:`k_\phi`
     - :math:`c` (kPa)
     - :math:`\phi` (deg)
   * - ``dry_sand``
     - 0%
     - 1.1
     - 0.99
     - 1528.43
     - 1.04
     - 28
   * - ``sandy_loam``
     - 15%
     - 0.7
     - 5.27
     - 1515.04
     - 1.72
     - 29
   * - ``clayey_soil``
     - 38%
     - 0.5
     - 13.19
     - 692.15
     - 4.14
     - 13
   * - ``clayey_soil_wet``
     - 55%
     - 0.7
     - 16.03
     - 1262.53
     - 2.07
     - 10
   * - ``heavy_clay``
     - 25%
     - 0.13
     - 12.70
     - 1555.95
     - 69.95
     - 34
   * - ``heavy_clay_wet``
     - 40%
     - 0.11
     - 1.84
     - 103.27
     - 20.69
     - 6
   * - ``lean_clay``
     - 22%
     - 0.2
     - 16.43
     - 1724.69
     - 68.95
     - 20
   * - ``lean_clay_wet``
     - 32%
     - 0.15
     - 1.52
     - 119.61
     - 13.79
     - 11
   * - ``lete_sand``
     -
     - 0.79
     - 102
     - 5301
     - 1.3
     - 31.1
   * - ``upland_sandy_loam``
     - 51%
     - 1.10
     - 74.6
     - 2080
     - 3.3
     - 33.7
   * - ``rubicon_sandy_loam``
     - 43%
     - 0.66
     - 6.9
     - 752
     - 3.7
     - 29.8
   * - ``north_gower_clayey_loam``
     - 46%
     - 0.73
     - 41.6
     - 2471
     - 6.1
     - 26.6
   * - ``grenville_loam``
     - 24%
     - 1.01
     - 0.06
     - 5880
     - 3.1
     - 29.8
   * - ``snow``
     -
     - 1.6
     - 4.37
     - 196.72
     - 1.03
     - 19.7
   * - ``tri1_lunar_simulant``
     -
     - 0.404
     - 6.60
     - 139.00
     - 0.36
     - 43.33 (K = 0.0103 m)

Track soil
----------

``"type": "track_soil"`` puts the soil under the shoes of a track. Each shoe
is a rectangle on its link: ``shoe_center``, ``shoe_normal`` (out of the shoe,
into the soil when it is on the ground), ``shoe_along`` (along the track,
across the grousers), ``shoe_length`` and ``shoe_width``, given in the
coordinates of the link. The ground is the plane :math:`z` = ``ground``, with
the soil below it (z down). The shoe is sampled at ``stations_along`` by
``stations_across`` points, each carrying an equal part of its area.

The pressure at a point sunk :math:`z` below the ground follows Bekker's law
while the soil is loaded further than ever before. Below the deepest sinkage
:math:`z_p` reached there it falls along a straight line back to zero, so that
the soil springs back by :math:`z_p` / ``unloading_ratio``, and a shoe that
is lifted leaves its rut behind. The deepest sinkage is a state that follows
the sinkage from below, over ``plastic_time``, and so there is no jump as the
soil is loaded and unloaded.

The soil also gives traction and side force. The shear on the shoe's plate is
the soil's shear strength under it, built up by the displacement :math:`\underline{j}` of the shoe
over the soil, :math:`1 - e^{-|\underline{j}|/k_j}`. The earth pressure of the soil in its way
adds to it: on the leading grouser along the track (of width ``shoe_width``
and the depth of the grouser's embedding, at most ``grouser_height``), and on
the side of the shoe across it, mobilized by the same factor. The displacement
:math:`\underline{j}` (two states, along and across the ground) is the slip of the shoe's
center over the ground while it is on the soil, and relaxes to zero over
``shear_time`` when the shoe is lifted.

.. code:: javascript

   {"type": "track_soil", "name": "soil", "soil": {"name": "lete_sand"},
    "shoes": ["link1", "link2"], "shoe_center": [0.095, 0, 0.082],
    "shoe_normal": [0, 0, 1], "shoe_along": [1, 0, 0],
    "shoe_length": 0.18, "shoe_width": 0.6, "grouser_height": 0.025}

Outputs, for each shoe, ``<shoe>_sinkage``, and for all the shoes
``normal_force``, ``traction`` (along the track) and ``side_force`` (across it),
in the ground plane.

.. shift-parameters:: shift/core/force_module/track_soil.cc
   :exclude: name, type, active

Example and test
----------------

* :source:`examples/terramechanics/wheel_test_bed.json` and
  ``wheel_test_bed_camber.json``: a Bekker-Wong wheel on a single-wheel test
  bed.
* Tests: :source:`test/test_terramechanics.cc`.
