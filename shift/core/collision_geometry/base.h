#ifndef SHIFT_COLLISIONGEOMETRY_BASE_H_
#define SHIFT_COLLISIONGEOMETRY_BASE_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ksl/SE3.h"
#include "shift/core/base.h"
#include "shift/core/frame.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {

namespace body {
class Base;
}
class Context;

namespace collision_geometry {

struct CadContactShape;

/*!
@brief Abstract CollisionGeometry base class

A collision geometry is a convex primitive attached to a body, used by the
contact detection in Context::DetectContacts. Each type implements an extreme
vertex support function, used by the GJK algorithm in gjk.h. It is given in one
of two ways when the model is loaded:

- by its dimensions (the radius of a sphere, the semi axes of an ellipsoid,
  the half extents of a box shaped convex hull), placed by the optional "pose"
  with respect to the body origin; no mesh is used.
- otherwise, fitted to the convex hull of the vertices of the body's meshes:
  all of its active meshes, or those named in "fit_to". A mesh named in
  fit_to may be inactive, which makes it a collision mesh that is not drawn.
  The fit places the geometry, so "pose" must not be given.

The collision geometry reference frame is located with respect to the body
origin by d_wrt_body_: the pose of a geometry given by its dimensions, or for
a fitted one the center of a sphere, the center and principal axes of an
ellipsoid, or the body origin for a convex hull.
*/
class Base
  : public shift::core::Base
  , public shift::core::parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name) {
    DeclareParameters();
  }
  Base()
    : shift::core::Base("") {
    DeclareParameters();
  }
  virtual ~Base() {}

  static const std::string get_array_key() {
    return "collision_geometry";
  }

  static const std::string get_default_type() {
    return "";
  }

  const std::string& get_type() {
    return type_;
  }

  virtual void ActivateObjects(bool parent_is_active) override;

  /*! \brief set the collision geometry from its dimensions, or fit it to the
   * vertices of the parent body's meshes; requires the active mesh geometry
   * to be loaded first */
  virtual void Load();
  virtual void Connect();

  /*! \brief fit the collision geometry to points in the body origin frame:
   * to their convex hull, or to all of them if the hull is degenerate (as
   * Load does with the mesh vertices) */
  void FitToPoints(const std::vector<ksl_vec3_t>& points);

  /*! \brief update the absolute displacement of the collision geometry
   * reference frame from the absolute displacement of the body origin */
  void UpdatePose(const ksl_SE3_t& body_origin_pose);

  /*! \brief the geometry as exact faces, edges and vertices, in the frame of
   * the geometry about CadContactShape::center, for contact with CAD geometry;
   * null if it has none. Set by SetExactShape once the geometry is loaded */
  const CadContactShape* get_exact_shape() const {
    return exact_shape_.get();
  }

  /*! \brief makes the geometry's exact shape; null if it has none */
  virtual std::shared_ptr<const CadContactShape> MakeExactShape() const {
    return nullptr;
  }

  /*! \brief what geometry of the same exact shape has in common, so that it
   * shares the shape */
  virtual const void* ExactShapeKey() const {
    return this;
  }

  void SetExactShape(std::shared_ptr<const CadContactShape> shape) {
    exact_shape_ = std::move(shape);
  }

  /*! \brief the radius its edges and vertices are taken to be rounded to, for
   * contact with CAD geometry; 0 for the default, 0.2% of its size */
  virtual double get_exact_edge_radius() const {
    return 0.0;
  }

  /*! \brief the deepest penetration taken for contact with CAD geometry; 0 if
   * not given */
  double get_max_depth() const {
    return max_depth_;
  }

  /*!
  @brief extreme vertex support function

  Finds the point on the collision geometry with the greatest projection along
  direction, expressed in the inertial frame.

  @param [in] direction search direction, expressed in the inertial frame,
  need not be normalized
  @param [out] local extreme point, expressed in the collision geometry
  reference frame
  @param [out] global extreme point, expressed in the inertial frame
  @param [in,out] hint index of the most recent extreme vertex, used by
  vertex based geometry to start its search
  */
  virtual void Support(const ksl_vec3_t& direction, ksl_vec3_t& local,
                       ksl_vec3_t& global, int& hint) const = 0;

  /*! \brief absolute displacement of the collision geometry reference frame,
   * valid after UpdatePose */
  const ksl_SE3_t& get_d_absolute() const {
    return d_absolute_;
  }

  /*! \brief displacement of the collision geometry reference frame with
   * respect to the body origin */
  const ksl_SE3_t& get_d_wrt_body() const {
    return d_wrt_body_;
  }

  /*! \brief radius of a sphere about the reference frame origin that bounds
   * the collision geometry, used to reject distant pairs */
  double get_bounding_radius() const {
    return bounding_radius_;
  }

      /*! \brief whether the geometry takes part in contact detection; geometry
   * with contact switched off is kept, and can be switched on again */
  bool is_contact_enabled() const {
    return contact_;
  }
  void set_contact_enabled(bool contact) {
    contact_ = contact;
  }

  /*! \brief pose of the geometry with respect to the body origin, as given
   * by the "pose" parameter */
  const ksl_SE3_t& get_pose() const {
    return *pose_.get_d_wrt_origin();
  }

  /*! \brief name of the contact material, empty for the default material
   * of the contact force module */
  const std::string& get_material() const {
    return material_;
  }

  /*! \brief whether the collision geometry was fitted to any vertices */
  bool is_loaded() const {
    return loaded_;
  }

protected:
  virtual void DeclareParameters() override;

  /*! \brief whether the dimensions of the geometry were given, so it is not
   * fitted to a mesh; throws if they are invalid */
  virtual bool HasDimensions() const {
    return false;
  }
  /*! \brief set the geometry from its dimensions in the frame pose, given
   * with respect to the body origin; sets d_wrt_body_ and bounding_radius_ */
  virtual void SetFromDimensions(const ksl_SE3_t&) {}

  /*! \brief fit the collision geometry to the vertices of the convex hull of
   * the body's mesh; faces index into vertices, and are empty if the hull is
   * degenerate (vertices then holds all mesh vertices) */
  virtual void Fit(const std::vector<ksl_vec3_t>& vertices,
                   const std::vector<unsigned int>& faces) = 0;

  /*! \brief copy the fitted data from collision geometry of the same type
   * fitted to the same mesh files */
  virtual void CopyFit(const Base& other) = 0;

  std::string type_;  //!< descriptive name of CollisionGeometry type
  /*! pose of a geometry given by its dimensions, wrt the body origin */
  shift::core::Frame pose_;
    /*! names of the body's mesh geometry to fit to, all active meshes if empty */
  std::vector<std::string> fit_to_;
    std::string material_; //!< contact material name, see get_material
  bool contact_ = true;   //!< see is_contact_enabled
  double max_depth_ = 0.0; //!< see get_max_depth

  ksl_SE3_t d_wrt_body_ = {{{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}}};
  ksl_SE3_t d_absolute_ = {{{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}}};
  double bounding_radius_ = 0.0;
  bool loaded_ = false;
  std::shared_ptr<const CadContactShape> exact_shape_;
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace collision_geometry
}
} // namespace shift

#endif
