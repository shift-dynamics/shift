#ifndef SHIFT_COLLISIONGEOMETRY_BASE_H_
#define SHIFT_COLLISIONGEOMETRY_BASE_H_

#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "shift/core/base.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {

namespace body {
class Base;
}
class Frame;
class Context;

namespace collision_geometry {

/*!
@brief Abstract CollisionGeometry base class

A collision geometry is a convex primitive attached to a body. It is fitted to
the vertices of the body's mesh geometry when the model is loaded, and is used
by the contact detection in Context::DetectContacts. Each type implements an
extreme vertex support function, used by the GJK algorithm in gjk.h.

The collision geometry reference frame is located with respect to the body
origin by d_wrt_body_, which each type sets when fitting itself: the center of
a sphere, the center and principal axes of an ellipsoid, or the body origin for
a convex hull.
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

  /*! \brief fit the collision geometry to the vertices of the parent body's
   * mesh geometry; requires the mesh geometry to be loaded first */
  virtual void Load();
  virtual void Connect();

  /*! \brief update the absolute displacement of the collision geometry
   * reference frame from the absolute displacement of the body origin */
  void UpdatePose(const ksl_SE3_t& body_origin_pose);

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

  /*! \brief whether the collision geometry was fitted to any vertices */
  bool is_loaded() const {
    return loaded_;
  }

protected:
  virtual void DeclareParameters() override;

  /*! \brief fit the collision geometry to the vertices of the convex hull of
   * the body's mesh; faces index into vertices, and are empty if the hull is
   * degenerate (vertices then holds all mesh vertices) */
  virtual void Fit(const std::vector<ksl_vec3_t>& vertices,
                   const std::vector<unsigned int>& faces) = 0;

  /*! \brief copy the fitted data from collision geometry of the same type
   * fitted to the same mesh files */
  virtual void CopyFit(const Base& other) = 0;

  std::string type_;  //!< descriptive name of CollisionGeometry type
  ksl_SE3_t d_wrt_body_ = {{{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}}};
  ksl_SE3_t d_absolute_ = {{{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}}};
  double bounding_radius_ = 0.0;
  bool loaded_ = false;
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace collision_geometry
}
} // namespace shift

#endif
