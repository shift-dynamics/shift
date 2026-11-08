.. _surfaces:
.. title:: Surfaces

Surfaces
========

Collision Geometry
------------------

The surfaces that take part in contact are distinct from the surfaces that are
drawn. Each body carries *collision geometry*, a list of shapes attached to
the body, and only these shapes are tested for contact. *Shift* represents the
surface of a shape in one of three ways:

* shapes defined by their dimensions: planes, spheres, ellipsoids, boxes,
  cylinders, capsules and convex hulls, each described by a closed form
  expression, and given either directly or by a fit to the triangle meshes
  that draw the body;
* the exact faces of parts read from STEP files, described by trimmed
  analytic and spline surfaces;
* terrain, described by an elevation map over the plane of a frame of the
  fixed body.

Each shape is defined in the coordinate system of its body, denoted
:math:`b`, and placed in the body by the shape's ``pose`` in the model file.
With the displacement of the body coordinate system relative to the fixed
reference frame :math:`0` written as
:math:`\Phi_{0b}^{0b}(R^{0b}, \underline{r}_{0b}^{0})`, the coordinates of a
point of the shape in the fixed reference frame are obtained from those in the
body coordinate system as:

.. math::
    :label: eq:shape_to_fixed

    \underline{p}^{0} = R^{0b}\,\underline{p}^{b} + \underline{r}_{0b}^{0}

where :math:`R^{0b}` transforms the coordinates of a vector from the body
coordinate system to the fixed reference frame, and
:math:`\underline{r}_{0b}^{0}` is the translation from the origin of the fixed
reference frame to the origin of the body, in the coordinates of the fixed
reference frame. All of the expressions that follow are written in the
coordinates of the fixed reference frame, and the superscript :math:`0` is
omitted from the vectors.


Shapes Defined by Their Dimensions
----------------------------------

The shapes defined by their dimensions are shown in Fig. :num:`dimensionshapes`. Each
shape is either given by its dimensions in the model file or fitted to the
vertices of the body's meshes, as described in :ref:`meshfitting`. With the exception of the plane, each
shape is bounded and convex, so that its contact with another shape is
determined by its *support point*, the point of the shape farthest along a
direction :math:`\underline{m}`:

.. math::
    :label: eq:support_point

    \underline{s}(\underline{m}) = \underset{\underline{p}}{\arg\max}\;
    \underline{m}\cdot\underline{p}

The search for the deepest point between two convex shapes, by the GJK and EPA
algorithms or by the closed forms available for particular pairs, requires
nothing more of a shape than Eq. :eq:`eq:support_point`.

.. _dimensionshapes:

.. figure:: figures/shapes.png
    :align: center
    :width: 700px

    Shapes defined by their dimensions: (a) sphere, (b) ellipsoid, (c) box,
    (d) cylinder, (e) capsule, (f) convex hull and (g) plane, with its normal
    pointing out of the solid

In the expressions below, :math:`\underline{c}` denotes the center of a shape
and :math:`\underline{x}`, :math:`\underline{y}` and :math:`\underline{z}`
denote the unit vectors along the axes of the shape, as placed by its pose or
by its fit, in the coordinates of the fixed reference frame.

Plane
*****

A plane (``"type": "plane"``) is the boundary of a half space. With the unit
normal :math:`\underline{n}` pointing out of the solid and the offset
:math:`o` of the plane along the normal, the solid comprises the points

.. math::
    :label: eq:plane

    \underline{n}\cdot\underline{p} \le o

The half space has no support point against :math:`\underline{n}`. The deepest
point of another shape in the half space is instead the support point of that
shape along :math:`-\underline{n}`, and its depth is
:math:`o - \underline{n}\cdot\underline{s}(-\underline{n})`.

Sphere and Ellipsoid
********************

A sphere (``"type": "sphere"``) of radius :math:`r` is the set of points

.. math::
    :label: eq:sphere

    \left|\underline{p} - \underline{c}\right| \le r,
    \qquad
    \underline{s}(\underline{m}) = \underline{c} + r\,\frac{\underline{m}}{|\underline{m}|}

An ellipsoid (``"type": "ellipsoid"``) with semi-axes :math:`a_x`,
:math:`a_y` and :math:`a_z` along its axes :math:`\underline{x}`,
:math:`\underline{y}` and :math:`\underline{z}` is described by the symmetric
shape matrix
:math:`G = a_x^2\,\underline{x}\,\underline{x}^T + a_y^2\,\underline{y}\,\underline{y}^T + a_z^2\,\underline{z}\,\underline{z}^T`:

.. math::
    :label: eq:ellipsoid

    (\underline{p} - \underline{c})^T G^{-1} (\underline{p} - \underline{c}) \le 1,
    \qquad
    \underline{s}(\underline{m}) = \underline{c} +
    \frac{G\,\underline{m}}{\sqrt{\underline{m}^T G\,\underline{m}}}

The sphere is the special case :math:`G = r^2 I`.

Box and Convex Hull
*******************

A convex hull (``"type": "convex_hull"``) is the smallest convex polyhedron
that contains a set of points, by default the vertices of the body's mesh. It
is computed by the quickhull algorithm :cite:`barber1996`, and its triangles
that lie in a common plane are merged into polygons. With the vertices
:math:`\underline{v}_i` and, for each face :math:`f`, the outward unit normal
:math:`\underline{n}_f` and the offset :math:`o_f`, the hull comprises the
points

.. math::
    :label: eq:hull

    \underline{n}_f\cdot\underline{p} \le o_f \quad \text{for every face } f,
    \qquad
    \underline{s}(\underline{m}) = \underset{\underline{v}_i}{\arg\max}\;
    \underline{m}\cdot\underline{v}_i

The support point is found by a search over the edges of the hull, from the
support point of the preceding evaluation to the neighboring vertex farthest
along :math:`\underline{m}`, until no neighbor lies farther. A box
(``"type": "box"``) with half extents :math:`h_x`, :math:`h_y` and :math:`h_z`
is the convex hull of its eight corners
:math:`\underline{c} \pm h_x\underline{x} \pm h_y\underline{y} \pm h_z\underline{z}`.

Cylinder and Capsule
********************

A cylinder (``"type": "cylinder"``) of radius :math:`r` and half length
:math:`h` has its axis along :math:`\underline{z}`. With the part of a vector
normal to the axis written as
:math:`\underline{m}_\perp = \underline{m} - (\underline{m}\cdot\underline{z})\,\underline{z}`,
the cylinder comprises the points

.. math::
    :label: eq:cylinder

    \left|(\underline{p} - \underline{c})\cdot\underline{z}\right| \le h,
    \quad
    \left|(\underline{p} - \underline{c})_\perp\right| \le r,
    \qquad
    \underline{s}(\underline{m}) = \underline{c} +
    h\,\mathrm{sgn}(\underline{m}\cdot\underline{z})\,\underline{z} +
    r\,\frac{\underline{m}_\perp}{|\underline{m}_\perp|}

A capsule (``"type": "capsule"``) is the set of points within the distance
:math:`r` of the segment from :math:`\underline{c} - h\underline{z}` to
:math:`\underline{c} + h\underline{z}`: a cylinder closed by a hemisphere at
each end. Its support point is

.. math::
    :label: eq:capsule

    \underline{s}(\underline{m}) = \underline{c} +
    h\,\mathrm{sgn}(\underline{m}\cdot\underline{z})\,\underline{z} +
    r\,\frac{\underline{m}}{|\underline{m}|}

The side of either shape is its axis grown by the radius, and a cylinder or a
capsule lying on a plane or on a face of a convex hull therefore touches it
along a line rather than at a point.


.. _meshfitting:

Shapes Fitted to Triangle Meshes
--------------------------------

The meshes that draw a body, read from Wavefront (``.obj``) and glTF
(``.gltf``, ``.glb``) files and divided into triangles, are not tested for
contact. A mesh file carries no unit: its coordinates are taken in the length
unit of the model, multiplied by the ``scale`` of the mesh. When the dimensions
of a collision shape are not given, the shape is fitted to the vertices of the
body's meshes, or to those of the meshes named in ``fit_to``. A mesh named in
``fit_to`` may be inactive, in which case it serves as a collision mesh that is
not drawn. The fit is computed once, when the model is loaded, and is shared by
all bodies that carry the same mesh files.

The fit begins with the convex hull of the vertices :math:`\underline{v}_i`,
Eq. :eq:`eq:hull`, and every shape other than the convex hull is fitted to the
vertices of the hull alone, since any shape that encloses the hull encloses
the mesh. The shapes fitted to the Stanford bunny, a mesh of 69 451 triangles,
are shown in Fig. :num:`meshfits`.

.. _meshfits:

.. figure:: figures/meshes.png
    :align: center
    :width: 700px

    A triangle mesh and the collision shapes fitted to it: (a) the mesh drawn
    as a wireframe, (b) a close up of its ears, and, in transparent purple
    about the shaded mesh, (c) the convex hull, (d) the ellipsoid, (e) the
    sphere, (f) the box, (g) the cylinder and (h) the capsule

The fits are as follows:

* convex hull: the hull itself, with the triangles that lie in a common plane
  merged into polygons;
* sphere: the sphere through the two vertices farthest apart along the
  principal axis of greatest spread, enlarged in turn to enclose each vertex
  outside of it :cite:`ritter1990`. The result is, in general, slightly larger
  than the smallest enclosing sphere;
* ellipsoid: the ellipsoid of least volume that encloses the vertices,

  .. math::
      :label: eq:mvee

      \min_{G,\,\underline{c}}\;\det G
      \quad\text{subject to}\quad
      (\underline{v}_i - \underline{c})^T G^{-1} (\underline{v}_i - \underline{c}) \le 1

  found by Khachiyan's algorithm :cite:`khachiyan1996,moshtagh2005`;
* box: the bounding box of the vertices along the principal axes, the
  eigenvectors of the covariance matrix

  .. math::
      :label: eq:covariance

      C = \frac{1}{n}\sum_{i=1}^{n}
      (\underline{v}_i - \bar{\underline{v}})(\underline{v}_i - \bar{\underline{v}})^T

  where :math:`\bar{\underline{v}}` is the mean of the :math:`n` vertices;
* cylinder and capsule: the axis :math:`\underline{z}` is the principal axis
  of greatest spread, the center is the middle of the extent of the vertices
  along the principal axes, and the radius is the greatest distance of a
  vertex from the axis. The half length of a capsule is the half extent along
  the axis less the radius, and its radius is then enlarged to enclose the
  vertices near its ends.

If the vertices lie in a plane, the hull is degenerate, and the shape is
fitted to all of the vertices instead.

Limitations
***********

The representation of a mesh by a fitted shape is subject to the following
limitations.

* Every fitted shape is convex. The concave parts of a mesh, such as the space
  between the ears of the bunny or a hole through a part, are filled, so that
  a peg cannot enter a hole and a body cannot rest in a hollow. A concave body
  is represented by several collision shapes, each fitted through ``fit_to``
  to a separate mesh of one convex part of the body, or by the exact surfaces
  of its STEP file.
* Every fitted shape encloses the mesh. Contact therefore begins before the
  drawn surfaces touch, wherever the shape stands off from the mesh; the gap
  is largest for the sphere, the box and the cylinder fitted to an elongated
  or irregular mesh.
* The faces of a convex hull are flat. A curved surface represented by a hull
  is a set of facets, across whose edges the normal of contact changes
  abruptly, and a hull fitted to a round mesh rolls as a polyhedron rather
  than as the round body. For their Hertz contact with the exact surfaces of
  STEP parts, the corners and edges of a hull are taken to be rounded with the
  radius ``edge_radius``, by default 0.2% of the size of the hull.
* The cost of the support point of a hull grows with the number of vertices
  visited in the search over its edges. The hull of a mesh of many fine
  triangles over a round surface has many vertices; a sphere, ellipsoid,
  cylinder or capsule describes such a surface at a fixed cost.


Exact Surfaces of STEP Parts
----------------------------

Parts designed in a CAD program are exchanged as STEP files, which describe
each part by the exact surfaces of its faces rather than by a mesh of
triangles. *Shift* reads the faces, edges and vertices of such parts and
evaluates their surfaces directly, both to draw them and to find their
contacts, so that a cylindrical bore remains a cylinder of the given radius
and a tooth flank remains the curve drawn by the designer.

Boundary Representation
***********************

A solid in a STEP file is described by its boundary, as shown in Fig.
:num:`breptopology`. The boundary of a solid is a closed *shell*, a set of *faces*
that encloses a volume. Each face is the part of a *surface* bounded by one or
more *loops*. Each loop is a closed chain of *edges*, and each edge is the part
of a *curve* that runs between two *vertices*. The surfaces and curves are the
geometry of the part, and the shells, faces, loops, edges and vertices are its
topology. A surface is shared by every face that lies on it, and an edge is
shared by the two faces that meet along it, each of which uses the edge in the
direction that runs counterclockwise about its own normal.

.. _breptopology:

.. figure:: figures/topology.png
    :align: center
    :width: 500px

    Faces, edges (yellow) and vertices (red) of a track link

The entities of the STEP file that describe the topology, and the parts of the
model that *Shift* builds from them, are listed in :ref:`Table 1 <step_topology>`.

.. _step_topology:

.. list-table:: Table 1: Topology of a STEP part
    :header-rows: 1
    :widths: 35 65

    * - STEP entity
      - Meaning
    * - ``MANIFOLD_SOLID_BREP``, ``BREP_WITH_VOIDS``
      - A solid bounded by an outer shell, and by inner shells around its voids
    * - ``SHELL_BASED_SURFACE_MODEL``
      - A set of shells that does not bound a solid
    * - ``CLOSED_SHELL``, ``OPEN_SHELL``
      - A connected set of faces
    * - ``ADVANCED_FACE``
      - A face: its bounds, its surface, and a flag ``same_sense`` that tells
        whether its normal is that of the surface or the opposite
    * - ``FACE_OUTER_BOUND``, ``FACE_BOUND``
      - The outer loop of a face, and the loops around its holes
    * - ``EDGE_LOOP``, ``VERTEX_LOOP``
      - A closed chain of oriented edges, or a single vertex, as at the apex
        of a cone
    * - ``ORIENTED_EDGE``
      - The use of an edge by a loop, in the direction of the edge or against
        it
    * - ``EDGE_CURVE``
      - An edge: its start and end vertices, its curve, and whether it runs
        along the curve's parameter or against it
    * - ``VERTEX_POINT``
      - A vertex, at a ``CARTESIAN_POINT``
    * - ``PCURVE``, ``SURFACE_CURVE``, ``SEAM_CURVE``
      - The curve of an edge in the parameters of a surface it lies on

A face is bounded in the parameters :math:`(u, v)` of its surface. When the
file is read, the loops of each face are mapped into the plane of
:math:`(u, v)`, from the curves of their edges in those parameters where the
file provides them and by projection onto the surface where it does not. The
resulting trim loops run counterclockwise around the outside of the face and
clockwise around its holes, so that the face is the region to the left of every
trim loop. On a closed surface, such as a cylinder, the loops are joined along
the seam where the periodic parameter returns to its start.

The outward normal of a face follows from the derivatives of its surface
:math:`\underline{p}(u, v)` with respect to the parameters,
:math:`\underline{p}_u = \partial\underline{p}/\partial u` and
:math:`\underline{p}_v = \partial\underline{p}/\partial v`:

.. math::
    :label: eq:face_normal

    \underline{n} = \pm\frac{\underline{p}_u \times \underline{p}_v}
    {\left|\underline{p}_u \times \underline{p}_v\right|}

with the positive sign where the face's ``same_sense`` is true.

The STEP File Format
********************

STEP, the Standard for the Exchange of Product model data, is the
international standard ISO 10303. The standard is divided into parts. The
generic resources, such as Part 42 :cite:`iso10303_42` for geometry and
topology, define the entities in a common form. The application protocols
select the entities used in a particular field: AP203 for configuration
controlled design, AP214 for automotive design, and AP242, which supersedes
both, for model based engineering. Part 21 :cite:`iso10303_21` defines the
text form of a file. Since the geometric and topological entities of all three
protocols are those of Part 42, *Shift* reads the three with the schema of
AP242, by means of the STEPcode library.

A Part 21 file consists of a header section and a data section. The header
names the file, the program that wrote it, and the schema of its data. The
data section is a list of entity instances, one per statement, of the form

.. code-block:: none

    #id=ENTITY_NAME(attribute, attribute, ...);

in which each attribute is a string (``'text'``), a number, an enumeration
(``.T.``, ``.MILLI.``), a reference to another instance (``#id``), a list in
parentheses, an omitted value (``$``) or a value derived from other
attributes (``*``). An instance of several entities at once, such as a
rational B-spline surface, is written as a list of the entities with their
own attributes, ``(ENTITY_A(...)ENTITY_B(...))``.

The file ``examples/cad/torus_patch.step``, written by
``scripts/make_surface_step.py``, describes a solid of a single face, the
patch of a torus shown in Fig. :num:`surfacetorus`. Its header and units are:

.. literalinclude:: ../../../examples/cad/torus_patch.step
    :language: none
    :lines: 1-8, 18-22

The units are assigned by the representation context ``#14``: lengths in
millimeters (``#18``) and plane angles in radians (``#19``). *Shift* converts
every length to meters and every angle to radians, including the angular
parameters of curves and surfaces, which a file in degrees gives in degrees.
The geometry and topology of the face follow:

.. literalinclude:: ../../../examples/cad/torus_patch.step
    :language: none
    :lines: 26-64

The surface ``#104`` is a torus placed by the coordinate system ``#103``, with
a major radius of 30 mm and a minor radius of 10 mm. The four edges
``#127`` to ``#133`` lie on the circles ``#107`` to ``#118``, and run between
the vertices ``#120`` to ``#126``. The loop ``#135`` uses the first two edges
in their own directions and the last two against them (``.F.``), so that it
runs counterclockwise about the outward normal. The face ``#137`` is the part
of the torus bounded by the loop, and the shell ``#138`` and the solid ``#139``
complete the part.

Placement of Geometry
*********************

Curves and surfaces are placed by a coordinate system,
``AXIS2_PLACEMENT_3D(name, location, axis, ref_direction)``. With the location
:math:`\underline{o}`, the axis :math:`\underline{a}` and the reference
direction :math:`\underline{b}`, the unit vectors of the coordinate system are

.. math::
    :label: eq:placement

    \underline{z} = \frac{\underline{a}}{|\underline{a}|}, \qquad
    \underline{x} = \frac{\underline{b} - (\underline{b}\cdot\underline{z})\,\underline{z}}
    {\left|\underline{b} - (\underline{b}\cdot\underline{z})\,\underline{z}\right|}, \qquad
    \underline{y} = \underline{z}\times\underline{x}

The axis defaults to :math:`(0, 0, 1)` and the reference direction to
:math:`(1, 0, 0)` when omitted. A surface of revolution is placed by an
``AXIS1_PLACEMENT``, a location and an axis only.

Curves
******

The curves of edges are given in one parameter :math:`t`. A line
(``LINE(name, point, vector)``) passes through the point
:math:`\underline{o}` along a ``VECTOR``, a direction and a magnitude whose
product :math:`\underline{d}` is the distance traveled per unit of the
parameter. A circle (``CIRCLE(name, position, radius)``) and an ellipse
(``ELLIPSE(name, position, semi_axis_1, semi_axis_2)``) lie in the plane of
:math:`\underline{x}` and :math:`\underline{y}` of their placement:

.. math::
    :label: eq:curves

    \begin{aligned}
    \text{line:}\quad & \underline{p}(t) = \underline{o} + t\,\underline{d} \\
    \text{circle:}\quad & \underline{p}(t) = \underline{o} +
        r\left(\cos t\,\underline{x} + \sin t\,\underline{y}\right) \\
    \text{ellipse:}\quad & \underline{p}(t) = \underline{o} +
        a_1\cos t\,\underline{x} + a_2\sin t\,\underline{y}
    \end{aligned}

A polyline is a chain of lines through its points, and a B-spline curve is
given by Eq. :eq:`eq:nurbs_curve` below.

Surfaces
********

The surfaces of faces are given in the two parameters :math:`(u, v)`. In the
following, :math:`\underline{o}`, :math:`\underline{x}`, :math:`\underline{y}`
and :math:`\underline{z}` are the location and unit vectors of a surface's
placement, Eq. :eq:`eq:placement`. In each figure, lines of constant
:math:`u` are drawn in blue and lines of constant :math:`v` in orange, inside
the trim loops of the face, and the boundary of the face is drawn in black.
Where a surface has a placement, its axes :math:`\underline{x}`,
:math:`\underline{y}` and :math:`\underline{z}` are drawn in red, green and blue.

Plane
^^^^^

``PLANE(name, position)`` is the plane of :math:`\underline{x}` and
:math:`\underline{y}`, with :math:`u` and :math:`v` the distances along them
and the normal :math:`\underline{x}\times\underline{y} = \underline{z}`:

.. math::
    :label: eq:step_plane

    \underline{p}(u, v) = \underline{o} + u\,\underline{x} + v\,\underline{y}

.. _surfaceplane:

.. figure:: figures/surface_plane.png
    :align: center
    :width: 420px

    A face of a plane

Cylinder
^^^^^^^^

``CYLINDRICAL_SURFACE(name, position, radius)`` has its axis along
:math:`\underline{z}` through :math:`\underline{o}`, with :math:`u` the angle
about the axis from :math:`\underline{x}`, periodic over :math:`2\pi`, and
:math:`v` the distance along the axis:

.. math::
    :label: eq:step_cylinder

    \underline{p}(u, v) = \underline{o} +
    r\left(\cos u\,\underline{x} + \sin u\,\underline{y}\right) + v\,\underline{z}

.. _surfacecylinder:

.. figure:: figures/surface_cylinder.png
    :align: center
    :width: 420px

    The side of a cylinder, with the seam at :math:`u = 0`

Cone
^^^^

``CONICAL_SURFACE(name, position, radius, semi_angle)`` has the radius
:math:`r` at :math:`v = 0` and opens with the semi-angle :math:`\alpha`; its
apex lies on the axis at :math:`v = -r / \tan\alpha`, where :math:`u` is
undefined:

.. math::
    :label: eq:step_cone

    \underline{p}(u, v) = \underline{o} +
    \left(r + v\tan\alpha\right)\left(\cos u\,\underline{x} + \sin u\,\underline{y}\right)
    + v\,\underline{z}

.. _surfacecone:

.. figure:: figures/surface_cone.png
    :align: center
    :width: 420px

    A cone, bounded by its apex and a circle

Sphere
^^^^^^

``SPHERICAL_SURFACE(name, position, radius)`` is parameterized by the
longitude :math:`u` about :math:`\underline{z}` and the latitude :math:`v`
from :math:`-\pi/2` to :math:`\pi/2`. At the poles, :math:`v = \pm\pi/2`, the
longitude is undefined:

.. math::
    :label: eq:step_sphere

    \underline{p}(u, v) = \underline{o} +
    r\cos v\left(\cos u\,\underline{x} + \sin u\,\underline{y}\right) +
    r\sin v\,\underline{z}

.. _surfacesphere:

.. figure:: figures/surface_sphere.png
    :align: center
    :width: 420px

    A hemisphere closing a tube, seen from below; the placement lies at the
    center of the sphere, inside the part

Torus
^^^^^

``TOROIDAL_SURFACE(name, position, major_radius, minor_radius)`` is swept by
a circle of the minor radius :math:`r_m` whose center travels on a circle of
the major radius :math:`r` about :math:`\underline{z}`. The parameter
:math:`u` is the angle about :math:`\underline{z}` and :math:`v` the angle
around the tube, both periodic:

.. math::
    :label: eq:step_torus

    \underline{p}(u, v) = \underline{o} +
    \left(r + r_m\cos v\right)\left(\cos u\,\underline{x} + \sin u\,\underline{y}\right)
    + r_m\sin v\,\underline{z}

.. _surfacetorus:

.. figure:: figures/surface_torus.png
    :align: center
    :width: 420px

    The outer half of a quarter of a torus, the face of ``torus_patch.step``

Surface of Revolution
^^^^^^^^^^^^^^^^^^^^^

``SURFACE_OF_REVOLUTION(name, swept_curve, axis_position)`` is swept by a
curve :math:`\underline{c}(v)` turning about the axis through
:math:`\underline{o}` along :math:`\underline{z}`. The parameter :math:`u` is
the angle of rotation and :math:`v` the parameter of the curve. With the
position of a point of the curve relative to the axis,
:math:`\underline{e} = \underline{c}(v) - \underline{o}`, the rotation by
Rodrigues' formula yields

.. math::
    :label: eq:step_revolution

    \underline{p}(u, v) = \underline{o} +
    (\underline{e}\cdot\underline{z})\,\underline{z} +
    \cos u\left(\underline{e} - (\underline{e}\cdot\underline{z})\,\underline{z}\right) +
    \sin u\;\underline{z}\times\underline{e}

.. _surfacerevolution:

.. figure:: figures/surface_revolution.png
    :align: center
    :width: 420px

    A quarter turn of a B-spline profile about :math:`\underline{z}`, the face
    of ``revolution_patch.step``

Surface of Linear Extrusion
^^^^^^^^^^^^^^^^^^^^^^^^^^^

``SURFACE_OF_LINEAR_EXTRUSION(name, swept_curve, extrusion_axis)`` is swept
by a curve :math:`\underline{c}(u)` translated along a ``VECTOR``, whose
direction and magnitude give :math:`\underline{d}`:

.. math::
    :label: eq:step_extrusion

    \underline{p}(u, v) = \underline{c}(u) + v\,\underline{d}

.. _surfaceextrusion:

.. figure:: figures/surface_extrusion.png
    :align: center
    :width: 420px

    The involute flank of a gear tooth, a B-spline curve extruded along the
    width of the gear

B-Spline Surface
^^^^^^^^^^^^^^^^

Free-form surfaces are B-spline surfaces :cite:`rogers2001,piegl1997`, given by
``B_SPLINE_SURFACE_WITH_KNOTS`` and, when the surface is rational, by the
complex instance that adds ``RATIONAL_B_SPLINE_SURFACE`` and its weights. The
surface of degree :math:`k` in :math:`u` and :math:`l` in :math:`v` is defined
by a net of :math:`(m + 1) \times (n + 1)` control points
:math:`\underline{q}_{ij}` with weights :math:`w_{ij}`:

.. math::
    :label: eq:nurbs_surface

    \underline{p}(u, v) =
    \frac{\sum_{i=0}^{m}\sum_{j=0}^{n} b_{i,k}(u)\,b_{j,l}(v)\,w_{ij}\,\underline{q}_{ij}}
         {\sum_{i=0}^{m}\sum_{j=0}^{n} b_{i,k}(u)\,b_{j,l}(v)\,w_{ij}}

where all weights equal 1 for a surface that is not rational, in which case the
denominator equals 1. The basis functions :math:`b_{i,k}` are piecewise
polynomials of degree :math:`k`, defined over the knot vector
:math:`\mu_0 \le \mu_1 \le \dots \le \mu_{m+k+1}` by the recurrence of Cox and
de Boor :cite:`deboor1972,rogers2001`:

.. math::
    :label: eq:basis

    \begin{aligned}
    b_{i,0}(u) &= \begin{cases} 1 & \text{if } \mu_i \le u < \mu_{i+1} \\
                               0 & \text{otherwise} \end{cases} \\
    b_{i,k}(u) &= \frac{u - \mu_i}{\mu_{i+k} - \mu_i}\,b_{i,k-1}(u) +
                  \frac{\mu_{i+k+1} - u}{\mu_{i+k+1} - \mu_{i+1}}\,b_{i+1,k-1}(u)
    \end{aligned}

in which a quotient with a zero denominator is taken as zero. The functions
:math:`b_{j,l}(v)` are defined in the same way over the knot vector
:math:`\nu_0 \le \dots \le \nu_{n+l+1}`. The surface is defined for
:math:`\mu_k \le u \le \mu_{m+1}` and :math:`\nu_l \le v \le \nu_{n+1}`. A STEP
file lists each distinct knot once, with its multiplicity, and *Shift* expands
the list into the full knot vector. A knot of multiplicity :math:`k + 1` at
each end makes the surface pass through the corner points of its net, as in
Fig. :num:`surfacenurbs`. A B-spline curve is the analogue in one parameter:

.. math::
    :label: eq:nurbs_curve

    \underline{p}(t) =
    \frac{\sum_{i=0}^{m} b_{i,k}(t)\,w_i\,\underline{q}_i}
         {\sum_{i=0}^{m} b_{i,k}(t)\,w_i}

With positive weights, each point of a B-spline surface lies within the convex
hull of the control points whose basis functions are nonzero there. Rational
weights allow the exact representation of conic sections: a quarter of a
circle is a rational curve of degree 2 with the control points at its ends and
at the intersection of its end tangents, and the weights :math:`1`,
:math:`\sqrt{2}/2` and :math:`1` :cite:`rogers2001`.

.. _surfacenurbs:

.. figure:: figures/surface_nurbs.png
    :align: center
    :width: 420px

    A rational B-spline surface with its control net in black, the face of
    ``rational_patch.step``

Evaluation
**********

The surfaces are evaluated from Eqs. :eq:`eq:step_plane` to
:eq:`eq:nurbs_surface` at every point at which they are needed, without
conversion to a mesh of triangles. The viewer evaluates the surfaces on the
graphics processor to draw the faces, and the contact detection evaluates the
surfaces, their derivatives :math:`\underline{p}_u` and
:math:`\underline{p}_v`, and their curvatures at the points of contact.


Terrain
-------

A terrain is a surface over the plane of a frame of the fixed body, described
by an elevation map, a grid of samples :math:`z_{k\ell}` spaced by
:math:`\Delta x` and :math:`\Delta y`, with the sample :math:`z_{k\ell}` at
:math:`x = k\,\Delta x`, :math:`y = \ell\,\Delta y`. The terrain is tested by
the force modules that query the surface under a point of a body, such as
those of tires and of terrain contact, and not by the collision geometry.

Bicubic Interpolation
*********************

The default interpolation, ``"interpolation_type": "bicubic"``, is the natural
cubic spline surface through the samples. It is a uniform cubic B-spline
surface, the case of Eq. :eq:`eq:nurbs_surface` with evenly spaced knots and
unit weights, over control points :math:`\zeta_{k\ell}` that are solved for
when the map is loaded. Within the cell :math:`(i, j)` of the grid, with the
local coordinates :math:`s = x/\Delta x - i` and :math:`t = y/\Delta y - j`
between 0 and 1, the elevation is

.. math::
    :label: eq:terrain

    z(x, y) = \sum_{a=0}^{3}\sum_{d=0}^{3}
    \beta_a(s)\,\beta_d(t)\,\zeta_{i+a-1,\,j+d-1}

with the uniform cubic basis functions

.. math::
    :label: eq:uniform_cubic

    \beta_0(s) = \frac{(1 - s)^3}{6}, \quad
    \beta_1(s) = \frac{3s^3 - 6s^2 + 4}{6}, \quad
    \beta_2(s) = \frac{-3s^3 + 3s^2 + 3s + 1}{6}, \quad
    \beta_3(s) = \frac{s^3}{6}

Since :math:`\beta_0(0) = \beta_2(0) = 1/6`, :math:`\beta_1(0) = 4/6` and
:math:`\beta_3(0) = 0`, the elevation at a sample is
:math:`(\zeta_{k-1} + 4\zeta_k + \zeta_{k+1})/6` along each axis. The
condition that the surface pass through every sample along a row of
:math:`n` samples is therefore

.. math::
    :label: eq:spline_interpolation

    \zeta_{k-1} + 4\,\zeta_k + \zeta_{k+1} = 6\,z_k, \qquad k = 0, \dots, n - 1

The control points :math:`\zeta_{-1}` and :math:`\zeta_n`, one cell beyond
the edges of the map, are extrapolated linearly:

.. math::
    :label: eq:natural_end

    \zeta_{-1} = 2\,\zeta_0 - \zeta_1, \qquad
    \zeta_n = 2\,\zeta_{n-1} - \zeta_{n-2}

which is the natural end condition, a second derivative of zero at the edges.
Under Eq. :eq:`eq:natural_end`, the first and last rows of Eq.
:eq:`eq:spline_interpolation` reduce to :math:`\zeta_0 = z_0` and
:math:`\zeta_{n-1} = z_{n-1}`, and the remaining rows form a tridiagonal,
diagonally dominant system, solved by the Thomas algorithm. Because the
surface is a tensor product, the system is solved along every row of the map
and then along every column. Samples that lie on a plane give control points
on the same plane, so that a plane is reproduced exactly, including the cells
at the edges of the map. Beyond the map, the control points of
Eq. :eq:`eq:natural_end` are held, so that the surface continues along its
slope at the edges for about a cell and then levels off.

A control point :math:`\zeta_{k\ell}` is weighted by :math:`\beta_{k-i+1}(s)`
in the cell :math:`i` and by :math:`\beta_{k-i}(s)` in the cell :math:`i + 1`.
At the boundary between the two cells, where :math:`s = 1` in the first and
:math:`s = 0` in the second, the basis functions and their first and second
derivatives satisfy

.. math::
    :label: eq:cubic_continuity

    \beta_a(1) = \beta_{a-1}(0), \qquad
    \beta_a'(1) = \beta_{a-1}'(0), \qquad
    \beta_a''(1) = \beta_{a-1}''(0), \qquad a = 1, 2, 3

while :math:`\beta_0` and its first two derivatives vanish at :math:`s = 1`,
as do :math:`\beta_3` and its first two derivatives at :math:`s = 0`. Each
control point therefore contributes the same elevation, slope and second
derivative on both sides of the boundary, and the same holds along
:math:`y`: the surface is :math:`C^2` continuous. The third derivatives are
constant within each cell and change between cells, so that the second
derivatives are continuous and piecewise linear.

Bilinear Interpolation
**********************

The bilinear interpolation, ``"interpolation_type": "bilinear"``, also passes
through the samples, and within a cell is

.. math::
    :label: eq:bilinear

    z(x, y) = (1 - s)(1 - t)\,z_{i,j} + s(1 - t)\,z_{i+1,j} +
    (1 - s)\,t\,z_{i,j+1} + s\,t\,z_{i+1,j+1}

Its elevation is continuous, but its slopes change abruptly at the boundaries
of the cells, so that its normal jumps there and its second derivatives are
undefined. Beyond the map, the surface continues with the samples of its
edges.

The two surfaces over one map are compared in Fig. :num:`terrainsurfaces`,
and along one row of samples in Fig. :num:`terrainsection`. A tire rolling
over the bicubic surface encounters a continuously changing curvature, so that
its forces change smoothly; over the bilinear surface, the forces change
abruptly wherever the tire crosses the boundary of a cell.

.. _terrainsurfaces:

.. figure:: figures/terrain.png
    :align: center
    :width: 700px

    Terrain over one elevation map, drawn by *Shift*: (a) the bicubic surface
    and (b) the bilinear surface, with the samples of the map in black and the
    boundaries of the cells in orange. Both surfaces pass through the samples.

.. _terrainsection:

.. figure:: figures/terrain_section.png
    :align: center
    :width: 560px

    Height, slope and second derivative along the row of samples
    :math:`y = 2\,\Delta y` of Fig. :num:`terrainsurfaces`. The vertical lines
    mark the boundaries of the cells. The bicubic slope and second derivative
    are continuous across the boundaries; the bilinear slope jumps at each
    boundary, and its second derivative is zero within the cells and undefined
    at the boundaries.
