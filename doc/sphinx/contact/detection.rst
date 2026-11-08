.. _detection:
.. title:: Contact Detection

Contact Detection
=================

Contact detection finds the pairs of collision geometries that touch, and for
each pair the points of contact, the depth of penetration at each point and
the normal along which the bodies are pushed apart. *Shift* detects contact at
every evaluation of the forces, before the force modules are evaluated, so that
the contact force module and any other force module that reads the contacts
see the state at which the forces are computed. Detection is divided into two
phases :cite:`ericson2005`:

* the *broad phase*, which discards, at little cost per pair, the pairs of
  geometries that are far apart, and leaves a list of candidate pairs;
* the *narrow phase*, which tests each candidate pair on the shapes
  themselves and computes its points of contact.

Of the :math:`n (n - 1) / 2` pairs of :math:`n` geometries, few are near one
another at any instant. The broad phase reduces the number of pairs tested by
the narrow phase from a number that grows as :math:`n^2` to one that grows
with the number of geometries near one another, which for bodies resting on a
surface or in a pile grows as :math:`n`.


Acceleration Structures
-----------------------

The structures used to find the pairs of objects near one another, in the
broad phase among bodies and in the narrow phase among the faces or triangles
of one body, are known as acceleration structures. Each encloses an object in a
simple *bounding volume*, a sphere, an axis aligned box or an oriented box,
whose overlap with another is tested by a few comparisons.

Sweep and prune
    The bounding volumes are projected onto an axis, and their intervals on
    the axis are sorted by their lower ends. Two volumes can overlap only if
    their intervals overlap, which a single pass over the sorted list finds
    :cite:`baraff1992,cohen1995`. Between successive evaluations the bodies
    move little, the order changes little, and an insertion sort restores it
    in nearly linear time. The method is efficient when the objects are spread
    along the axis, and degrades when many objects share an interval, as a
    stack of bodies does along the vertical axis.

Uniform grids and spatial hashing
    Space is divided into cells of equal size, and each object is entered into
    the cells its bounding volume overlaps. Objects are paired only with those
    in the same cells. A spatial hash stores the occupied cells of an unbounded
    grid in a hash table :cite:`teschner2003`. Grids are efficient for objects
    of similar size, and less so where the sizes differ widely, since the size
    of the cell suits only one size of object.

Octrees and k-d trees
    Space is divided recursively, into eight octants of a cube or by planes
    normal to the coordinate axes, until each cell holds few objects
    :cite:`bentley1975,samet2006`. The division adapts to the distribution of
    the objects, at the cost of rebuilding or updating the tree as the objects
    move.

Bounding volume hierarchies
    The objects themselves, rather than space, are divided recursively, and
    each node of the tree holds a bounding volume of the objects below it. Two
    trees are tested from their roots down, and a pair of subtrees whose
    volumes do not overlap is discarded with all of its descendants. The
    bounding volumes in use include spheres :cite:`hubbard1996`, oriented boxes
    :cite:`gottschalk1996`, axis aligned boxes :cite:`vandenbergen1997` and
    discrete oriented polytopes, bounded by planes normal to a fixed set of
    directions :cite:`klosowski1998`. A hierarchy over the parts of a rigid
    body is built once, in the coordinate system of the body, and remains valid
    as the body moves.

A surface given as a regular grid of samples, such as the elevation map of a
terrain, needs none of these structures: the grid is its own index, and the
cell under a point is found directly from the point's coordinates.

The narrow phase between two convex shapes is commonly computed by the
algorithm of Gilbert, Johnson and Keerthi (GJK) :cite:`gilbert1988`, which
finds whether the origin lies in the Minkowski difference of the two shapes,
the set of the differences
:math:`\underline{p}_b - \underline{p}_a` of their points, by a sequence of
simplices whose vertices are differences of support points,
Eq. :eq:`eq:support_point`. Where the shapes overlap, the expanding polytope
algorithm (EPA) :cite:`vandenbergen2004` expands the final simplex of GJK
towards the boundary of the Minkowski difference to find the depth of
penetration and the normal. Both algorithms require nothing of a shape but its
support point.


Broad Phase in *Shift*
----------------------

*Shift* uses sweep and prune over bounding spheres. Each collision geometry
has a bounding sphere about the origin of its coordinate system, whose radius
:math:`\rho` is the greatest distance of a point of the geometry from that
origin. For the broad phase, the sphere is enlarged by a margin
:math:`\delta = \rho / 4`. The broad phase:

#. records the center :math:`\underline{c}_i` of each geometry
   :math:`i`, in the coordinates of the fixed reference frame;
#. selects the axis of the fixed reference frame along which the centers are
   most spread, the axis of greatest variance;
#. sorts the geometries by the lower ends
   :math:`c_{i} - \rho_i - \delta_i` of their enlarged spheres along that axis,
   by an insertion sort from the order of the previous broad phase;
#. sweeps the sorted list, and for each geometry tests the geometries whose
   intervals begin before its own ends. A pair whose enlarged spheres overlap,

   .. math::
       :label: eq:sphere_overlap

       \left|\underline{c}_i - \underline{c}_j\right| \le
       \rho_i + \delta_i + \rho_j + \delta_j

   becomes a candidate pair.

Planes are unbounded and are not swept: each plane is paired with every
bounded geometry. Geometries of the same body, or of bodies joined rigidly,
are never paired, nor are bodies excluded from contact with one another, such
as bodies joined by a joint whose geometries overlap at the joint.

The broad phase of a group of bunnies, each with a convex hull fitted to its
mesh, is shown in Fig. :num:`broadphase`.

.. _broadphase:

.. figure:: figures/broadphase.png
    :align: center
    :width: 700px

    The broad phase of nine bunnies, computed by *Shift*. Each bunny is drawn
    within its bounding sphere (purple), centered at the origin of the
    coordinate system of its convex hull, and the sphere enlarged by its
    margin (grey). The candidate pairs are joined in red: the three pairs of
    the group of three, the pair whose bounding spheres overlap, and the pair
    whose spheres overlap only with their margins. Below, the intervals of the
    enlarged spheres along the axis of the sweep, :math:`x`, are drawn in the
    order of their lower ends. Each bunny is also a candidate with the floor,
    a plane, which is not drawn.

The broad phase is not repeated at every evaluation. A pair that is not a
candidate was separated, at the last broad phase, by more than the sum of the
enlarged radii. As long as no geometry has moved by more than its margin
:math:`\delta_i` since then, the pair remains separated by more than
:math:`\rho_i + \rho_j`, and its geometries cannot touch. The candidate list is
therefore kept until the center of some geometry leaves the sphere of radius
:math:`\delta_i` about its recorded center. Since the bounding sphere is
centered at the origin of the geometry's coordinate system, rotation of the
geometry about that origin does not leave its bounding sphere.

Narrow Phase in *Shift*
-----------------------

Each candidate pair is first tested on its bounding spheres without the
margin, or for a plane on the height of the sphere's center above the plane.
Pairs that pass are tested by the method of their pair of types:

* a plane and any shape: the support point of the shape along the inward
  normal of the plane, Eq. :eq:`eq:plane`, or a line of contact for a
  cylinder or capsule lying on the plane;
* two spheres, a sphere and an ellipsoid, a sphere and a convex hull, and two
  ellipsoids: closed forms, or the minimization of a function of one
  variable;
* two convex hulls, and a convex hull and an ellipsoid: GJK, followed either
  by the contact of the features of the shapes about the normal of the last
  accepted step, while that contact persists, so that the normal does not
  jump between minima of nearly equal depth, or by EPA for a new contact;
* any other pair of convex shapes: GJK, and EPA for the depth and the normal;
* exact surfaces of STEP parts: the contact of the faces, edges and vertices
  of the parts, described below.

After the deepest point is found, the contact of two convex shapes is extended
to a *manifold*: the polygon of contact of two faces, the line of contact of a
cylinder or capsule on a face, or the single deepest point. The force module
applies its contact law at each point of the manifold.

The state of each pair is kept from one evaluation to the next. GJK begins from
the newest vertex of its previous simplex, and the support point of a convex
hull is found by a search over its edges that begins at the support vertex of
the previous evaluation :cite:`cameron1997`. Between nearby states, both
searches end after few steps.

For a pair of STEP parts, an axis aligned box tree over the faces of each part
is built once, when the model is loaded, in the coordinate system of the part.
The boxes of the faces of the second part are transformed into the coordinate
system of the first, each as the axis aligned box that bounds the transformed
box, and each is tested against the tree of the first part. The pairs of faces
whose boxes overlap are then tested on their surfaces, by Newton's method on
the parameters :math:`(u, v)` of the faces, or by slicing a line of contact
along its length.

The candidate pairs are independent of one another. When the tests of the
previous evaluation took longer than 100 μs, they are distributed among the
processor's threads; the results are collected in the order of the candidates,
so that they do not depend on the number of threads.


Coordinate Systems
------------------

Each part of the detection is computed in the coordinate system in which it is
least expensive. Every collision geometry is described in a coordinate system
fixed in its body, as in Eq. :eq:`eq:shape_to_fixed`; the coordinate systems
of the two geometries of a pair are denoted :math:`i` and :math:`j`.

.. list-table:: Table 2: Coordinate systems of contact detection
    :header-rows: 1
    :widths: 45 55

    * - Computation
      - Coordinate system
    * - Bounding spheres, sweep and prune, the tests of bounding spheres
      - Fixed reference frame :math:`0`: the centers
        :math:`\underline{c}_i = \underline{r}_{0i}^{0}`
    * - Support point of a shape, Eq. :eq:`eq:support_point`
      - The direction :math:`\underline{m}^{0}` is transformed into the
        geometry, :math:`\underline{m}^{i} = R^{0i\,T}\,\underline{m}^{0}`;
        the support point is found from the vertices or dimensions stored in
        :math:`i`, and transformed back by Eq. :eq:`eq:shape_to_fixed`
    * - GJK and EPA simplices, the Minkowski difference, the manifold
      - Fixed reference frame :math:`0`; the vertices of the simplex are also
        kept in the coordinates of each geometry, to restart GJK at the next
        evaluation
    * - Contact of STEP parts: box trees, Newton's method on the faces, lines
        of contact
      - The coordinate system :math:`i` of the first part. The second part is
        transformed into it by the relative displacement
        :math:`\Phi_{ij}^{ij} = \Phi_{0i}^{0i\,-1}\,\Phi_{0j}^{0j}`, and the
        points and normals are transformed into the fixed reference frame
        when the pair is complete
    * - Contact of a STEP part and a plane
      - The coordinate system of the part, into which the plane's normal and
        point are transformed
    * - Terrain queries of the tire and terrain contact modules
      - The coordinate system of the terrain, into which each point is
        transformed before its elevation and slopes are evaluated
    * - Points and normals given to the force modules
      - Fixed reference frame :math:`0`, and the points also in the coordinates
        of each geometry, for the curvatures of the surfaces there

Computing in the coordinate system of a geometry allows its vertices, face
planes, box tree and surfaces to be stored once, when the model is loaded,
and never updated as the body moves: only the direction of a query, or the
relative displacement of the other geometry, is transformed. Computing the
broad phase and the simplices in the fixed reference frame allows all
geometries, and both geometries of a pair, to be compared in one coordinate
system.


Contact Detection on the Graphics Processor
-------------------------------------------

*Shift* computes contact detection on the central processor, distributing the
poses of the geometries and the tests of the candidate pairs among its threads.
The graphics processor is used only to draw the model. A graphics processor
executes one operation on many data at once, with thousands of arithmetic units
working in parallel; it attains its throughput, the amount of work completed
per unit time, when the work consists of many independent tasks of equal
length that perform the same operations. The parts of contact detection differ
in how well they meet that condition.

Broad phase
    Sweep and prune, spatial hashing and bounding volume hierarchies have been
    implemented on graphics processors, with the sort or the tree construction
    distributed over all of the objects
    :cite:`legrand2007,liu2010,karras2012`. For models of thousands of bodies,
    such as a pile of fitted meshes, the broad phase could be moved to the
    graphics processor. In *Shift* the broad phase is repeated only when a
    geometry leaves its margin, and the sort starts from a nearly sorted list,
    so that its cost is small compared with that of the narrow phase in the
    models measured to date.

Narrow phase of primitive shapes
    The tests of spheres, ellipsoids, boxes, cylinders, capsules and small
    convex hulls are short computations of similar length, and many of them
    can be computed at once, one pair per arithmetic unit. A model with
    thousands of such pairs in contact would gain from computing them on the
    graphics processor.

Narrow phase of STEP parts
    The contact of exact surfaces traverses box trees, iterates Newton's method
    a varying number of times, and slices lines of contact; the work of each
    pair of faces differs in length and in its sequence of operations. Such
    work maps poorly onto a graphics processor, and remains on the central
    processor.

Terrain queries
    A terrain requires no search. The cell under a point is found directly
    from the coordinates of the point in the terrain's coordinate system,
    :math:`i = \lfloor x/\Delta x \rfloor` and
    :math:`j = \lfloor y/\Delta y \rfloor`, and the elevation and slopes
    follow from the sixteen control points about the cell,
    Eq. :eq:`eq:terrain`. The cost of a query is constant, independent of the
    size of the map, and amounts to about a hundred arithmetic operations, less
    than the cost of transferring the point to the graphics processor and the
    result back. Terrain queries therefore remain on the central processor.

Three constraints limit the gain. First, contact is detected at every
evaluation of the forces, one or more times per integration step, and each
evaluation would transfer the poses to the graphics processor and the contacts
back; for a model with tens of contact pairs, the time of the transfer and of
starting the computation on the graphics processor can exceed that of the
detection on the central processor. Second, the integration is computed in double precision, which
many graphics processors execute at a small fraction of the rate of single
precision, while single precision is insufficient for depths of penetration of
micrometers on bodies of meters. Third, the results must not depend on the
order in which the parallel tasks complete, as they do not on the central
processor. Moving detection to the graphics processor is therefore
worthwhile for models of many bodies of primitive shapes, and for many terrain
queries, rather than for models of a few STEP parts.
