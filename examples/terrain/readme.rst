Terrain
=======

This directory contains *Shift* `JSON <https://shift-dynamics.github.io/shift/model-file/model-file.html>`_ model files that demonstrate the display of terrains (i.e. digital elevation maps). In *Shift*, terrains are loaded from raster image data specified in the floating point TIFF format. Tires drive on them (see the `vehicle <../vehicle>`_ and `raptor <../raptor>`_ models).

.. image:: terrain.png

Diffuse, specular and bump map textures can be applied to terrain to provide realistic visual detail. ``bricard_asphalt.json``, ``bricard_concrete.json`` and ``bricard_sand.json`` use textures built into the viewer, and the viewer's ray traced mode (*View*, *Ray traced*) adds shadows.
