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

The datasets grow as the simulation runs, and the file is flushed after
every row, so it can be read while the simulation is still running.

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
