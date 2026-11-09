#ifndef SHIFT_COLLISIONGEOMETRY_CAD_H_
#define SHIFT_COLLISIONGEOMETRY_CAD_H_

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "shift/core/cad/brep.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/collision_geometry/fitting.h"
#include "shift/core/collision_geometry/gjk.h"
#include "ksl/SE3.h"
#include "ksl/vec3.h"

namespace shift {
namespace core {

namespace geometry {
struct CadAsset;
class Cad;
} // namespace geometry

namespace collision_geometry {

/*! \brief transformation of a point of a STEP file (meters) to the frame of
 * a collision geometry (model length units): \f$\underline{p}' = d \, (\text{scale} \cdot \underline{p})\f$ */
struct CadPlacement {
  ksl_SE3_t d = {{{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}}};
  double scale = 1.0;

  ksl_vec3_t Apply(const cad::Vec3& p) const;
  /*! a derivative or direction of the file, in the frame of the geometry
   * (scaled) */
  ksl_vec3_t ApplyVector(const cad::Vec3& v) const;
  /*! the point of the file at p' */
  cad::Vec3 Invert(const ksl_vec3_t& p) const;
};

/*! \brief axis aligned box */
struct CadBox {
  ksl_vec3_t lo = {{HUGE_VAL, HUGE_VAL, HUGE_VAL}};
  ksl_vec3_t hi = {{-HUGE_VAL, -HUGE_VAL, -HUGE_VAL}};

  void Add(const ksl_vec3_t& p);
  void Add(const CadBox& b);
  void Pad(double margin);
  bool Overlaps(const CadBox& b) const;
  /*! the box bounding this one transformed by d */
  CadBox Transformed(const ksl_SE3_t& d) const;
};

/*! \brief a face of CAD geometry, placed in the frame of the collision
 * geometry */
struct CadContactFace {
  int face = -1;      //!< in the model of the asset
  int placement = -1; //!< in CadContactShape::placements
  /*! the trim loops of the face as polygons in (u, v) (outer loops
   * counterclockwise, holes clockwise), and their bounds */
  std::vector<std::vector<cad::Vec2>> trims;
  cad::Vec2 lo{0.0, 0.0};
  cad::Vec2 hi{0.0, 0.0};
  /*! the sides of the trim polygons, (a, b) as (u_a, v_a, u_b, v_b), and
   * for each of bands of equal height in v from lo to hi the sides that
   * reach into it, so an inside test crosses only the sides of its band */
  std::vector<std::array<double, 4>> sides;
  std::vector<std::vector<int>> bands;
  bool periodic_u = false;
  bool periodic_v = false;
  /*! points inside the trims, (u, v) and in the frame of the geometry, with
   * the outward unit normals there: starting points of the searches */
  std::vector<cad::Vec2> sample_uv;
  std::vector<ksl_vec3_t> sample_points;
  std::vector<ksl_vec3_t> sample_normals;
  /*! the points of the trim polygons, in the frame of the geometry */
  std::vector<ksl_vec3_t> boundary_points;
  CadBox box; //!< bounds the face, in the frame of the geometry
  std::vector<int> edges;    //!< in CadContactShape::edges
  std::vector<int> vertices; //!< in CadContactShape::vertices
};

/*! \brief a face tessellated in (u, v) for Cad::RayCastGrid: a grid of nu by
 * nv cells over the bounds of its trims, fine enough that the surface is
 * within error of the planes of its triangles (two in each cell) */
struct CadFaceMesh {
  int nu = 1;
  int nv = 1;
  std::vector<cad::Vec2> uv;      //!< (nu + 1) * (nv + 1), u fastest
  std::vector<ksl_vec3_t> points; //!< there, frame of the geometry
  double error = 0.0;             //!< largest distance found off the planes
};

/*! \brief an edge of CAD geometry, placed in the frame of the geometry */
struct CadContactEdge {
  int edge = -1; //!< in the model of the asset
  int placement = -1;
  std::vector<int> faces; //!< in CadContactShape::faces
  bool sharp = false; //!< a convex edge between two faces at an angle
  bool smooth = false; //!< a seam of a face, or two faces meeting smoothly
  /*! points along the edge, without its ends, with their parameters */
  std::vector<double> sample_t;
  std::vector<ksl_vec3_t> sample_points;
  CadBox box; //!< bounds the edge, in the frame of the geometry
};

/*! \brief a vertex of CAD geometry, placed in the frame of the geometry */
struct CadContactVertex {
  ksl_vec3_t point = {{0.0, 0.0, 0.0}};
  std::vector<int> faces; //!< in CadContactShape::faces
};

/*! \brief node of the box tree over the faces of a shape: an inner node has
 * two children, a leaf the faces order[first, first + count) */
struct CadBoxNode {
  CadBox box;
  int left = -1;
  int right = -1;
  int first = 0;
  int count = 0;
};

/*!
@brief the faces, edges and vertices of the bodies of a CAD geometry, placed
in the frame of a collision geometry, with a box tree over the faces; shared
by collision geometry of the same file and scale
*/
struct CadContactShape {
  std::shared_ptr<const geometry::CadAsset> asset;
  std::vector<CadPlacement> placements; //!< of the asset's bodies
  std::vector<CadContactFace> faces;
  std::vector<CadContactEdge> edges;
  std::vector<CadContactVertex> vertices;
  std::vector<CadBoxNode> nodes; //!< nodes[0] is the root
  std::vector<int> order;
  /*! points on the faces and edges, whose convex hull approximates the
   * shape for Support */
  std::vector<ksl_vec3_t> hull_points;
  CadBox box;
  double size = 0.0; //!< diagonal of box
  /*! origin of the frame of the geometry in the body origin frame: the
   * center of the bounds */
  ksl_vec3_t center = {{0.0, 0.0, 0.0}};

  const cad::Model& model() const;
  /*! faces whose boxes overlap box, appended to faces */
  void Query(const CadBox& box, std::vector<int>& faces) const;
  /*! point of a face at (u, v) in the frame of the geometry, with its
   * derivatives with respect to u and v there */
  ksl_vec3_t Point(int face, const cad::Vec2& uv, ksl_vec3_t* du = nullptr,
                   ksl_vec3_t* dv = nullptr) const;
  /*! outward unit normal of a face at (u, v), in the frame of the geometry */
  ksl_vec3_t Normal(int face, const cad::Vec2& uv) const;
  /*! mean curvature of a face at (u, v), positive where the face is convex
   * (its outward normal turning away from the solid) */
  double MeanCurvature(int face, const cad::Vec2& uv) const;
  /*! curvature of a face at (u, v) in the directions of the orthonormal
   * tangents \f$\underline{t}_1\f$ and \f$\underline{t}_2\f$ (frame of the
   * geometry): \f$c = (c_{11}, c_{12}, c_{22})\f$, the curvature in direction
   * \f$\underline{t}\f$ being \f$\underline{t}^T c \, \underline{t}\f$, positive
   * where convex; zero on a plane */
  void Curvature(int face, const cad::Vec2& uv, const ksl_vec3_t& t1,
                 const ksl_vec3_t& t2, double c[3]) const;
  /*! (u, v) of the point of a face's surface nearest p, in the frame of the
   * geometry; false if not found */
  bool Project(int face, const ksl_vec3_t& p, cad::Vec2& uv,
               const cad::Vec2* guess = nullptr) const;
  /*! whether (u, v) is inside the face's trims, with periodic parameters
   * taken in any period */
  bool Inside(int face, const cad::Vec2& uv) const;
  /*! the pieces \f$[t_0, t_1]\f$ of the segment from a to b in \f$(u, v)\f$,
   * \f$\underline{a} + t (\underline{b} - \underline{a})\f$ with
   * \f$t \in [0, 1]\f$, that are inside the face's trims; the segment is taken
   * in the period of periodic parameters nearest the face. With inset, a
   * share of the face's span, an end within that share of the face's bounds
   * is first brought inside them by it, so that a segment along the edge of
   * the face is inside it */
  std::vector<std::array<double, 2>> Clip(int face, const cad::Vec2& a,
                                          const cad::Vec2& b,
                                          double inset = 0.0) const;
};

/*!
@brief exact contact of CAD geometry: the faces of the body's STEP geometry
(geometry type "cad"), with their analytic and NURBS surfaces trimmed by
their edges

Contact between two CAD geometries (see CadCadContact) and between CAD
geometry and a plane (CadPlaneContact) is computed on the exact surfaces:
a box tree over each geometry's faces finds the pairs of faces that may
touch; two cylinders, and a cylinder on a plane, touch along a line at any
angle, sliced over the length where they overlap, so that a line tilting out
of parallel carries its load towards one end, and becomes a point as two
cylinders cross, or the rim of a cylinder tilting up on a plane (with no
jump in the force between); a NURBS or extruded face touches a cylinder or a
plane the same way where the gap between them is curved mainly in one direction,
as a cam on its roller or flat follower, each section of the line solved on the
face, and across a seam between faces; other pairs of faces touch at points
found by Newton's method on the surfaces; and edges and vertices touch faces where no
pair of their faces does. Points outside a face's trims are not on the face.

Contact with a sphere or an ellipsoid (CadRoundContact) is exact too. With its
center \f$\underline{c}\f$ and its shape matrix
\f$G = R \, \mathrm{diag}(a_x^2, a_y^2, a_z^2) R^T\f$, the point of the shape whose
outward normal is \f$\underline{m}\f$ has the closed form
\f$\underline{e}(\underline{m}) = \underline{c} + G \underline{m} /
\sqrt{\underline{m}^T G \underline{m}}\f$, so that a point of contact with a face
is the root of the two equations in \f$(u, v)\f$ alone,

\f[
(\underline{e}(-\underline{n}) - \underline{p}) \cdot \underline{p}_u = 0, \quad
(\underline{e}(-\underline{n}) - \underline{p}) \cdot \underline{p}_v = 0,
\f]

with \f$\underline{p}\f$ the point of the face and \f$\underline{n}\f$ its outward
normal there: the shape has no parameters of its own, nor poles. For a sphere
this is the point of the face closest to \f$\underline{c}\f$. A face can touch at
several points, and the curvature of the gap is the sum of the face's and that of
the shape, \f$\underline{s}^T G^{-1} \underline{t} / |G^{-1} (\underline{e} -
\underline{c})|\f$ in the tangents \f$\underline{s}\f$ and \f$\underline{t}\f$.
An edge or vertex touches where no face next to it does, at the point deepest in
the shape. A point counts only with the center of the shape on the outer side of
the face, so a shape does not touch a thin wall from behind.

Boxes, convex hulls and cylinders have exact shapes of their own
(Base::get_exact_shape), faces of planes and cylinders with edges and vertices, so
that they touch CAD geometry as other CAD geometry does (CadExactContact): a corner
on a face, a side along a line, a face's edge on an edge. A capsule is the side of a
cylinder, with no edges, and a sphere at each end: its line of contact gives way to
the hemisphere as it tilts. The edges and vertices of these shapes are rounded to
their edge_radius, 0.002 of their size by default, and those of a capsule's cylinder
blend into its ends at its radius. A hull of points in a plane or along a line is
thickened to 1e-4 of their extent, so that it has faces. Only a hull of a single
point has no exact shape: it touches the convex hull of points on the faces of the
CAD geometry, tested by GJK.

Two sharp edges that cross touch at their closest points, solved on the curves
themselves (CadContactEdge::sharp): the normal \f$\underline{n}\f$ is along the
cross product of their tangents, and they touch if \f$\underline{n}\f$ is among the
outward normals of the first edge's faces, \f$-\underline{n}\f$ among the second's,
and the first edge is beyond the second along \f$\underline{n}\f$. Such a crossing
replaces the contact of each edge with the faces of the other, whose depth is to a
face that ends there.

Near a sharp edge of a face, within the Hertz radius of a round shape's point, the
stiffness goes over to that of the edge. A straight edge along a cylinder, of a
primitive or of another CAD part, touches it along a line.

A straight smooth edge (CadContactEdge::smooth), a seam of a cylinder, say, is
no contact feature on a flat face: it has no corner, and no deepest point there.

The contact force module (force_module::Contact) applies Hertz contact at each
point: of a line for line contacts, of the ellipse of the faces' principal
curvatures at a point (HertzEllipseFactor), and of edges and vertices
rounded to edge_radius.

The geometry is the body's CAD geometry, named by fit_to or the first CAD
geometry of the body; its frame is the body origin frame, translated to the
center of the geometry's bounds.
*/
class Cad : public Base {
public:
  Cad()
    : Base() {
    type_ = "cad";
    DeclareParameters();
  }
  Cad(std::string name)
    : Base(name) {
    type_ = "cad";
    DeclareParameters();
  }
  virtual ~Cad() override {}

  static std::unique_ptr<Cad> Create();

  static bool is_registered() {
    return registered_;
  }

  virtual void Load() override;

  virtual void Support(const ksl_vec3_t& direction, ksl_vec3_t& local,
                       ksl_vec3_t& global, int& hint) const override;

  /*! \brief on the exact faces: the faces whose boxes the ray crosses (by
   * the box tree), each solved for the points where the ray meets its
   * surface by Newton's method from the samples of the face nearest the ray,
   * counted where they are inside the face's trims; see Base::RayCast */
  bool RayCast(const ksl_vec3_t& origin, const ksl_vec3_t& direction,
               double max_distance, double& distance,
               ksl_vec3_t& normal) const override;

  /*!
  @brief the first points at which the rays of a grid reach the exact faces,
  as RayCast gives, without casting each ray: each face is tessellated once
  in (u, v) (see CadFaceMesh), and its triangles are projected onto the grid.
  At each ray a triangle covers, the point interpolated in the triangle
  starts Newton's method for the point where the ray meets the surface,
  counted where it is inside the face's trims. Every crossing of a ray with a
  face is so found from the triangles over it, also where a ray meets one face
  twice (a cylinder). See Base::RayCastGrid.
  */
  void RayCastGrid(const RayGrid& grid, double* distances) const override;

  const CadContactShape& get_shape() const {
    return *shape_;
  }

  /*! \brief radius to which edges and vertices are rounded for their Hertz
   * contact, model length units */
  double get_edge_radius() const {
    return edge_radius_;
  }

  /*! \brief deepest penetration taken for contact; deeper points are taken
   * to be on the far side of a body */

protected:
  virtual void DeclareParameters() override;
  virtual void Fit(const std::vector<ksl_vec3_t>& vertices,
                   const std::vector<unsigned int>& faces) override;
  virtual void CopyFit(const Base& other) override;

private:
  static bool registered_;
  double edge_radius_ = 0.0;
  std::shared_ptr<const CadContactShape> shape_;
  /*! the faces tessellated for RayCastGrid, made on its first call */
  mutable std::shared_ptr<const std::vector<CadFaceMesh>> face_meshes_;
};

/*!
@brief contact of two CAD geometries, see Cad; sets the pair's contact data
and its manifold, each point with its normal and its curvature or length
(see ContactPoint)
*/
bool CadCadContact(ContactPair& pair);

/*! @brief contact of CAD geometry and a plane (either may be pair.a) */
bool CadPlaneContact(ContactPair& pair);

/*! @brief contact of CAD geometry and a sphere or an ellipsoid (either may be
 * pair.a), see Cad; sets the pair's contact data */
bool CadRoundContact(ContactPair& pair);

/*! @brief contact of CAD geometry and a geometry with an exact shape (either
 * may be pair.a), see Cad and Base::get_exact_shape; sets the pair's contact
 * data */
bool CadExactContact(ContactPair& pair);

/*! @brief builds the contact shape of a CAD asset at a scale (exposed for
 * tests) */
std::shared_ptr<const CadContactShape>
MakeCadContactShape(std::shared_ptr<const geometry::CadAsset> asset,
                    double scale);

/*! @brief the exact shape of a convex hull: a plane face for each of its
 * polygons, with their shared edges and vertices; null without polygons */
std::shared_ptr<const CadContactShape>
MakeHullShape(const ConvexHullData& hull);

/*! @brief the exact shape of a cylinder about the z axis, centered at the
 * origin: its side and, if caps, its ends, each rim in four arcs so that a
 * cylinder standing on a face stands on four points. Without caps it is the
 * side alone, with no edges or vertices, as a capsule's is */
std::shared_ptr<const CadContactShape>
MakeCylinderShape(double radius, double half_length, bool caps);

} // namespace collision_geometry
} // namespace core
} // namespace shift

#endif
