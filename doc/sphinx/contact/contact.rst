.. _contact_theory:
.. title:: Contact Modeling

Contact Modeling
================

The following sections describe the methods *Shift* uses to detect contact
between bodies and to compute the resulting forces. The description begins with
the representation of the surfaces of bodies, which ranges from shapes defined
by a small number of dimensions to the exact faces of parts read from STEP
files. The contact laws applied at the points of contact are described with the
:doc:`contact force module <../force-modules/contact>`. The last section
describes the discrete element method, which models a granular material as a
large number of grains in contact with each other and with the bodies of a
model.

.. toctree::
   :maxdepth: 2

   surfaces
   detection
   dem
