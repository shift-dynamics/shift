Environments
------------

Places to put robots and vehicles in, to share among models: each is a
terrain with its surface materials, scenery strewn over it from
`../scenery <../scenery>`_ and an environment (sky, sun, water), written by
a Python script that also gives the root keys a model needs to stand in it.

* `Sand beach on the sea <beach>`_: ``beach_parts()`` of ``make_beach.py``;
  used by ``examples/beach``.
* `Gravel rally stage through a forest <forest>`_: ``forest_parts()`` and
  ``driver()`` of ``make_forest.py``; used by ``examples/forest``.

A model's script imports the environment's script and copies the root keys
into the model, their files named from the model's directory, for example::

  sys.path.insert(0, "../../assets/environments/forest")
  import make_forest
  parts = make_forest.forest_parts(directory_of_the_model_file)
  for key in ("frames", "environment", "surface_materials", "terrain",
              "scenery"):
      model[key] = parts[key]
