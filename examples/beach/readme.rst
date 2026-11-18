Strandbeest on the beach
------------------------

``strandbeest_beach.json`` is the Strandbeest of `../strandbeest <../strandbeest>`_ walking along the sand beach of `../../assets/environments/beach <../../assets/environments/beach>`_: its feet sink into the deformable wet sand of the beach's strip and leave their prints, by the sea, with boulders, rocks and stones about it.

``make_strandbeest_beach.py`` writes it, from ``../strandbeest/strandbeest.json`` and ``beach_parts()`` of the beach's ``make_beach.py``: the beach in place of the walker's sand square and lights, and a soil contact force module for the feet. Regenerate it from this directory, after ``../strandbeest/make_strandbeest.py``::

  python3 make_strandbeest_beach.py
