.. _logging:
.. title:: Logging

Logging
=======

Loggers write the results of a simulation to files, one row every
``time_step`` of the integrator. They are listed in ``loggers`` in
``model_parameters``; a model may have several, for example a CSV file to
look at and an HDF5 file to analyze.

.. code:: javascript

   "loggers": [
     {"type": "csv", "csv_logfile": "excavator.csv",
      "timestamp_logfile": false},
     {"type": "hdf5", "logfile": "excavator.h5", "timestamp_logfile": true}
   ]

With ``timestamp_logfile`` the date and time are added to the file name
(``excavator-2026-11-03_20-40-24.h5``), so that runs do not overwrite each
other.

Keys of every logger:

.. shift-parameters:: shift/core/logger/base.cc

CSV
---

``"type": "csv"`` writes a text table with a header row. A joint or force
module is named by its fully qualified name, the names of the assemblies around
it and its own, separated by double underscores:
``<assembly>__<subassembly>__<name>``. A name
may be used only once at one level of the model, and a model that repeats one
does not load (see :ref:`model-file`), so two hydraulic cylinders of the same name in different assemblies have different
columns. Its columns are:

* ``time``;
* for each joint, its coordinates, velocities and accelerations:
  ``<assembly>__<joint>__q_<i>``, ``<assembly>__<joint>__dq_<i>`` and
  ``<assembly>__<joint>__ddq_<i>``;
* ``rms_constraint_error``, the root mean square error of the loop closure
  equations, and ``gravitational_potential_energy``, ``kinetic_energy`` and
  ``total_energy``;
* the outputs of the force modules, ``<assembly>__<module>__<output>``, such as
  ``excavator__boom_cylinder_1__head_pressure`` or
  ``accessory_drive__belt__span_0_tension``.

.. shift-parameters:: shift/core/logger/csv.cc

HDF5
----

``"type": "hdf5"`` writes an HDF5 file with the datasets

.. list-table::
   :header-rows: 1

   * - dataset
     - contents
   * - ``/time``
     - the time, one value per row
   * - ``/states/q``, ``/states/dq``
     - the coordinates and velocities, one row per output step
   * - ``/state_meta_data/generalized_variables``
     - each joint's offsets into them
   * - ``/force_modules/<assembly>__<module>/<output>``
     - each output of each force module, one value per row
   * - ``/terrain/<name>``
     - for each deformable terrain (``"type": "scm"``), the nodes whose level
       changed at each output step (see below)

The datasets grow as the simulation runs, and the file is flushed after
every row, so it can be read while the simulation is still running.

A deformable terrain's grid can have millions of nodes, but only the few
under the bodies change at each step. So ``/terrain/<name>`` stores only the
nodes that changed, not the whole grid at every row:

.. list-table::
   :header-rows: 1

   * - entry
     - contents
   * - attributes ``node_count``, ``x0``, ``y0``, ``spacing``
     - ``[nx, ny]``. Node ``(i, j)`` is at ``x0 + i spacing``,
       ``y0 + j spacing`` in the terrain frame.
   * - attribute ``pose``
     - the terrain frame in the inertial frame, a 4 x 4 matrix
   * - ``undisturbed_z``
     - ``ny`` x ``nx``: z (down) of the undisturbed surface at each node,
       written once
   * - ``index``, ``level``
     - the changes of every row, one after another: the node ``j nx + i``
       (int32) and its new level (float32), the depth of the surface below
       the undisturbed one (negative where soil is piled)
   * - ``row_start``
     - one per row of ``/time``: the first change of that row. The changes
       of row ``r`` run from ``row_start[r]`` up to ``row_start[r + 1]``, or
       to the end for the last row.

The first row lists every node whose level is not 0. To get the surface at
row ``r``, start from a grid of zeros and apply the changes of rows 0 to
``r`` in order:

.. code:: python

   import h5py, numpy as np

   with h5py.File("track.h5") as f:
       g = f["/terrain/sand"]
       nx, ny = g.attrs["node_count"]
       start, index, level = g["row_start"][:], g["index"][:], g["level"][:]
       r = len(start) - 1  # the last row
       levels = np.zeros(nx * ny, np.float32)
       end = start[r + 1] if r + 1 < len(start) else len(index)
       # the last change of each node up to row r
       nodes, last = np.unique(index[:end][::-1], return_index=True)
       levels[nodes] = level[:end][::-1][last]
       z = g["undisturbed_z"][:] + levels.reshape(ny, nx)

.. shift-parameters:: shift/core/logger/hdf5.cc

Reading the results
-------------------

The CSV file opens in any spreadsheet or plotting program. In Python, for
example:

.. code:: python

   import csv, h5py

   with open("excavator.csv") as f:
       rows = list(csv.DictReader(f, skipinitialspace=True))
   pressure = [float(r["excavator__boom_cylinder_1__head_pressure"]) for r in rows]

   with h5py.File("excavator.h5") as f:
       t = f["/time"][:]
       force = f["/force_modules/excavator__boom_cylinder_1/force"][:]

Force module outputs
--------------------

.. list-table::
   :header-rows: 1

   * - module
     - outputs
   * - :doc:`../force-modules/bushing`, :doc:`../force-modules/rubber-mount`,
       :doc:`../force-modules/beam`
     - ``pair_<k>_fx``, ``_fy``, ``_fz``, ``_mx``, ``_my``, ``_mz``
   * - :doc:`../force-modules/belt`
     - ``span_<k>_tension``; with friction, ``pulley_<k>_slip``
   * - :doc:`../force-modules/hydraulic-cylinder`
     - ``head_pressure``, ``rod_pressure``, ``force``, ``extension``,
       ``opening``, ``head_flow``, ``rod_flow``
