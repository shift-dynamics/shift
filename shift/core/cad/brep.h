#ifndef SHIFT_CORE_CAD_BREP_H_
#define SHIFT_CORE_CAD_BREP_H_

#include <array>
#include <string>
#include <vector>

namespace shift {
namespace core {

/*!
@brief CAD models: parts described by the exact surfaces of their faces and
the curves of their edges (a boundary representation), as read from STEP files

All lengths are in meters and all angles in radians, whatever the units of
the file. Geometry is held in pools of the Model (curves, surfaces), and
topology refers to it by index, so that faces and edges sharing a surface or
curve share its entry.
*/
namespace cad {

using Vec2 = std::array<double, 2>;
using Vec3 = std::array<double, 3>;

/*! @brief right-handed orthonormal coordinate system: origin and axes */
struct Frame {
  Vec3 origin{0.0, 0.0, 0.0};
  Vec3 x{1.0, 0.0, 0.0};
  Vec3 y{0.0, 1.0, 0.0};
  Vec3 z{0.0, 0.0, 1.0};
};

/*! @brief rigid transformation of coordinates: p' = rotation p + translation
 * (rotation row-major) */
struct Transform {
  std::array<double, 9> rotation{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  Vec3 translation{0.0, 0.0, 0.0};

  Vec3 Apply(const Vec3& p) const;
  /*! this transformation after b: (this * b)(p) = this(b(p)) */
  Transform operator*(const Transform& b) const;
  Transform Inverse() const;
  /*! the transformation of coordinates in frame f to those of its parent */
  static Transform FromFrame(const Frame& f);
};

/*!
@brief non-uniform rational B-spline curve in D dimensions

knots is the full knot vector (each knot repeated by its multiplicity), of
points.size() + degree + 1 values. weights is empty for a non-rational curve.
The curve is defined for parameters from knots[degree] to
knots[points.size()].
*/
template <int D>
struct NurbsCurve {
  int degree = 0;
  std::vector<double> knots;
  std::vector<std::array<double, D>> points;
  std::vector<double> weights;

  bool rational() const {
    return !weights.empty();
  }
};

/*!
@brief non-uniform rational B-spline surface

points holds u_count rows of v_count points: the point (i, j), i along u and
j along v, is points[i * v_count + j], and likewise for weights (empty for a
non-rational surface). u_knots and v_knots are full knot vectors.
*/
struct NurbsSurface {
  int u_degree = 0;
  int v_degree = 0;
  int u_count = 0;
  int v_count = 0;
  std::vector<double> u_knots;
  std::vector<double> v_knots;
  std::vector<Vec3> points;
  std::vector<double> weights;

  bool rational() const {
    return !weights.empty();
  }
};

/*!
@brief curve in D dimensions (3 for the curves of edges, 2 for curves in the
parameters of a surface)

- line: p(t) = origin + t direction (direction is not of unit length: its
  length is the distance per unit parameter)
- conic (circle or ellipse): p(t) = origin + a cos t + b sin t, t in radians;
  a and b are the semi-axes of a circle or ellipse in 3D, and remain
  conjugate semi-diameters when a curve in a surface's parameters is scaled
  by different factors along each parameter
- nurbs: see NurbsCurve
*/
template <int D>
struct Curve {
  enum class Type { kLine, kConic, kNurbs };
  Type type = Type::kLine;
  std::array<double, D> origin{};
  std::array<double, D> direction{}; //!< line
  std::array<double, D> a{};         //!< conic
  std::array<double, D> b{};         //!< conic
  NurbsCurve<D> nurbs;
  int step_id = 0; //!< entity of the STEP file, 0 if none
};

using Curve2 = Curve<2>;
using Curve3 = Curve<3>;

/*!
@brief surface of a face, with its parameters (u, v)

In the coordinates of the frame position (o, x, y, z), with lengths in
meters and angles in radians:

- plane: p = o + u x + v y
- cylinder: p = o + radius (cos u x + sin u y) + v z
- cone: p = o + (radius + v tan semi_angle) (cos u x + sin u y) + v z
- sphere: p = o + radius cos v (cos u x + sin u y) + radius sin v z
- torus: p = o + (radius + minor_radius cos v) (cos u x + sin u y)
  + minor_radius sin v z
- nurbs: see NurbsSurface
- revolution: the curve (curves[curve]) at parameter v, turned by the angle
  u about the axis through position.origin along position.z
- extrusion: the curve (curves[curve]) at parameter u, translated by v
  extrusion
*/
struct Surface {
  enum class Type {
    kPlane,
    kCylinder,
    kCone,
    kSphere,
    kTorus,
    kNurbs,
    kRevolution,
    kExtrusion
  };
  Type type = Type::kPlane;
  Frame position;
  double radius = 0.0;
  double minor_radius = 0.0;
  double semi_angle = 0.0;
  NurbsSurface nurbs;
  int curve = -1; //!< swept curve of a revolution or extrusion
  Vec3 extrusion{0.0, 0.0, 0.0};
  int step_id = 0;
};

/*! @brief red, green and blue from 0 to 1 */
struct Color {
  float r = 0.0f;
  float g = 0.0f;
  float b = 0.0f;
  bool set = false;
};

struct Vertex {
  Vec3 point{0.0, 0.0, 0.0};
  int step_id = 0;
};

/*! @brief curve of an edge in the parameters of one of the surfaces it
 * lies on */
struct Pcurve {
  int surface = -1;
  Curve2 curve;
  /*! parameters of curve at the edge's start and end vertices (set by
   * Repair) */
  std::array<double, 2> range{0.0, 0.0};
  /*! the pcurve's parameter equals the edge's (a linear map of
   * Edge::range onto range gives points within Edge::tolerance) */
  bool same_parameter = false;
  /*! computed by Repair, not read from the file */
  bool computed = false;
};

/*!
@brief edge between two vertices, along a curve

same_sense tells whether the edge runs from start to end along increasing
parameters of the curve. curve is -1 when the edge is given only by its
curves in the parameters of its surfaces (pcurves).
*/
struct Edge {
  int start = -1;
  int end = -1;
  int curve = -1;
  bool same_sense = true;
  std::vector<Pcurve> pcurves;
  /*! parameters of the curve (of pcurves[0] if there is no curve) at the
   * start and end vertices, increasing if same_sense (set by Repair) */
  std::array<double, 2> range{0.0, 0.0};
  bool has_range = false;
  /*! largest distance found between the edge's vertices, curve and pcurves
   * (set by Repair) */
  double tolerance = 0.0;
  int step_id = 0;
};

/*!
@brief use of an edge by a loop of a face: the edge runs from start to end if
orientation is true, from end to start otherwise. pcurve is the index of the
edge's pcurve on the face's surface, -1 if the file gave none.
*/
struct Coedge {
  int edge = -1;
  bool orientation = true;
  int pcurve = -1;
};

/*!
@brief boundary of a face: a closed chain of coedges, or a single vertex
(vertex >= 0, e.g. the apex of a cone). The loop is used reversed when
orientation is false.
*/
struct Loop {
  std::vector<Coedge> coedges;
  int vertex = -1;
  bool orientation = true;
  bool outer = false;
};

/*!
@brief piece of a trim loop in the parameters (u, v) of a face's surface:
the pcurve edges[edge].pcurves[pcurve] from parameter s0 to s1, translated by
offset (whole periods of a periodic surface), or a straight segment from a to
b that is not an edge:

- kJoin: closes a gap between edges smaller than the tolerance
- kSingular: runs along a singular point of the surface (the apex of a cone,
  a pole of a sphere), where one parameter is arbitrary
- kSeam: runs along the surface between two loops that wrap around a periodic
  surface (e.g. the two circles bounding a tube), cutting the face open
- kGap: closes a gap between edges larger than the tolerance (a defect of
  the file)
*/
struct TrimCurve {
  enum class Kind { kEdge, kJoin, kSingular, kSeam, kGap };
  Kind kind = Kind::kEdge;
  int edge = -1;
  int pcurve = -1;
  double s0 = 0.0;
  double s1 = 0.0;
  Vec2 offset{0.0, 0.0};
  Vec2 a{0.0, 0.0};
  Vec2 b{0.0, 0.0};
};

/*! @brief closed chain of trim curves, each starting where the previous one
 * ends: counterclockwise in (u, v) if outer, clockwise for a hole */
struct TrimLoop {
  std::vector<TrimCurve> curves;
  bool outer = false;
};

/*!
@brief bounded region of a surface; its normal is that of the surface if
same_sense, opposite otherwise

trims, set by Repair, bounds the face in the parameters of its surface: the
face is the region to the left of every trim loop (inside the outer loops and
outside the holes), with the outer loops first.
*/
struct Face {
  int surface = -1;
  bool same_sense = true;
  std::vector<Loop> loops;
  std::vector<TrimLoop> trims;
  Color color;
  int step_id = 0;
};

struct Shell {
  std::vector<int> faces;
  bool closed = false;
  int step_id = 0;
};

/*!
@brief a solid (its first shell bounds it from outside, any others are voids)
or a set of open or closed shells that are not a solid
*/
struct Body {
  std::string name;
  std::vector<int> shells;
  bool solid = false;
  Color color;
  int step_id = 0;
};

/*!
@brief shape of a part or an assembly: bodies, and child shapes placed by
transformations from their coordinates to this shape's
*/
struct Shape {
  struct Child {
    int shape = -1;
    Transform transform;
    /*! the child is another representation of the same part, in the same
     * coordinates (e.g. its geometry, apart from its placements), and its
     * name is not part of the path of its bodies */
    bool same_part = false;
  };
  std::string name;
  std::vector<int> bodies;
  std::vector<Child> children;
  int step_id = 0;
};

/*! @brief a body of a model in the coordinates of a root shape, see
 * Model::Flatten */
struct PlacedBody {
  int body = -1;
  int shape = -1;  //!< shape the body belongs to
  int root = -1;   //!< root shape it is placed in
  std::string path; //!< shape names from the root, separated by '/'
  Transform transform;
};

struct Model {
  std::vector<Curve3> curves;
  std::vector<Surface> surfaces;
  std::vector<Vertex> vertices;
  std::vector<Edge> edges;
  std::vector<Face> faces;
  std::vector<Shell> shells;
  std::vector<Body> bodies;
  std::vector<Shape> shapes;
  std::vector<int> roots; //!< shapes that are not children of others
  /*! problems found while reading, each with the entity it concerns */
  std::vector<std::string> warnings;

  /*! every body of every root shape, with its transformation to the root */
  std::vector<PlacedBody> Flatten() const;
};

} // namespace cad
} // namespace core
} // namespace shift

#endif
