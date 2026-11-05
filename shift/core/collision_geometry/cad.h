#ifndef SHIFT_COLLISIONGEOMETRY_CAD_H_
#define SHIFT_COLLISIONGEOMETRY_CAD_H_

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "shift/core/cad/brep.h"
#include "shift/core/collision_geometry/base.h"
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
 * a collision geometry (model length units): p' = d (scale p) */
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

/*! \brief an edge of CAD geometry, placed in the frame of the geometry */
struct CadContactEdge {
  int edge = -1; //!< in the model of the asset
  int placement = -1;
  std::vector<int> faces; //!< in CadContactShape::faces
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
   * tangents t1 and t2 (frame of the geometry): c = (c11, c12, c22), the
   * curvature in direction t being t^T c t, positive where convex; zero on
   * a plane */
  void Curvature(int face, const cad::Vec2& uv, const ksl_vec3_t& t1,
                 const ksl_vec3_t& t2, double c[3]) const;
  /*! (u, v) of the point of a face's surface nearest p, in the frame of the
   * geometry; false if not found */
  bool Project(int face, const ksl_vec3_t& p, cad::Vec2& uv,
               const cad::Vec2* guess = nullptr) const;
  /*! whether (u, v) is inside the face's trims, with periodic parameters
   * taken in any period */
  bool Inside(int face, const cad::Vec2& uv) const;
  /*! the pieces [t0, t1] of the segment from a to b in (u, v), a + t (b - a)
   * with t in [0, 1], that are inside the face's trims; the segment is taken
   * in the period of periodic parameters nearest the face */
  std::vector<std::array<double, 2>> Clip(int face, const cad::Vec2& a,
                                          const cad::Vec2& b) const;
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
jump in the force between); other pairs of faces touch at points found by
Newton's method on the surfaces; and edges and vertices touch faces where no
pair of their faces does. Points outside a face's trims are not on the face. With other
collision geometry (spheres, ellipsoids, convex hulls) a CAD geometry is the
convex hull of points on its faces, tested by GJK.

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
  double get_max_depth() const {
    return max_depth_;
  }

protected:
  virtual void DeclareParameters() override;
  virtual void Fit(const std::vector<ksl_vec3_t>& vertices,
                   const std::vector<unsigned int>& faces) override;
  virtual void CopyFit(const Base& other) override;

private:
  static bool registered_;
  double edge_radius_ = 0.0;
  double max_depth_ = 0.0;
  std::shared_ptr<const CadContactShape> shape_;
};

/*!
@brief contact of two CAD geometries, see Cad; sets the pair's contact data
and its manifold, each point with its normal and its curvature or length
(see ContactPoint)
*/
bool CadCadContact(ContactPair& pair);

/*! @brief contact of CAD geometry and a plane (either may be pair.a) */
bool CadPlaneContact(ContactPair& pair);

/*! @brief builds the contact shape of a CAD asset at a scale (exposed for
 * tests) */
std::shared_ptr<const CadContactShape>
MakeCadContactShape(std::shared_ptr<const geometry::CadAsset> asset,
                    double scale);

} // namespace collision_geometry
} // namespace core
} // namespace shift

#endif
