#ifndef SHIFT_COLLISIONGEOMETRY_PLANE_H_
#define SHIFT_COLLISIONGEOMETRY_PLANE_H_

#include <memory>

#include "shift/core/collision_geometry/base.h"

namespace shift {
namespace core {
namespace collision_geometry {

/*!
@brief half space collision geometry, e.g. the ground

The solid side is behind the plane: the points \f$\underline{x}\f$ of the pose frame
(the body frame unless a "pose" is given) with
\f$\underline{n} \cdot \underline{x} \le o\f$, for the normal \f$\underline{n}\f$
and the offset \f$o\f$. normal points out of the solid, into free space; with
shift's z axis down, flat ground through the body origin has normal
[0, 0, -1] and offset 0. The plane is given by its parameters rather than
fitted to the body's mesh. Attach it to a body joined to the fixed frame by a
rigid joint.

The collision geometry reference frame has the orientation of the pose, with
its origin on the plane at \f$o \, \underline{n}\f$ in the pose.

Contact of a plane with another shape is found from the other shape's
support point against the normal, its point deepest in the half space (see
analytic_contact.h).
*/
class Plane : public Base {
public:
  Plane()
    : Base() {
    type_ = "plane";
    DeclareParameters();
  }
  Plane(std::string name)
    : Base(name) {
    type_ = "plane";
    DeclareParameters();
  }
  virtual ~Plane() override {}

  static std::unique_ptr<Plane> Create();

  static bool is_registered() {
    return registered_;
  }

  /*! \brief set up the plane from its parameters; no mesh is needed */
  virtual void Load() override;

  /*! \brief set the plane, in the body frame, as Load does from the
   * parameters */
  void SetPlane(const ksl_vec3_t& normal, double offset);

  /*! \brief set the pose of the plane with respect to the body origin, as
   * the "pose" parameter does; the normal and offset are kept */
  void set_pose(const ksl_SE3_t& pose);

  /*! \brief unit normal out of the solid, in the inertial frame, valid after
   * UpdatePose */
  ksl_vec3_t get_normal() const;

  /*! \brief unit normal out of the solid and a point on the plane, in the
   * body frame */
  ksl_vec3_t get_normal_wrt_body() const;
  const ksl_vec3_t& get_point_wrt_body() const {
    return d_wrt_body_.t;
  }

  /*! \brief point on the plane, in the inertial frame, valid after
   * UpdatePose */
  const ksl_vec3_t& get_point() const {
    return d_absolute_.t;
  }

  /*! a half space has no extreme point along directions into the solid;
   * returns the point of the plane under the reference frame origin. Pairs
   * with a plane are tested analytically and never call this. */
  virtual void Support(const ksl_vec3_t& direction, ksl_vec3_t& local,
                       ksl_vec3_t& global, int& hint) const override;

protected:
  virtual void DeclareParameters() override;
  virtual void Fit(const std::vector<ksl_vec3_t>& vertices,
                   const std::vector<unsigned int>& faces) override;
  virtual void CopyFit(const Base& other) override;

private:
  static bool registered_;
  ksl_vec3_t normal_ = {{0.0, 0.0, -1.0}}; //!< body frame, from the input
  double offset_ = 0.0;
  ksl_vec3_t unit_normal_ = {{0.0, 0.0, -1.0}}; //!< body frame, normalized
};

} // namespace collision_geometry
} // namespace core
} // namespace shift

#endif
