#ifndef SHIFT_COLLISIONGEOMETRY_CYLINDER_H_
#define SHIFT_COLLISIONGEOMETRY_CYLINDER_H_

#include <memory>

#include "shift/core/collision_geometry/base.h"

namespace shift {
namespace core {
namespace collision_geometry {

/*!
@brief cylinder collision geometry

A right circular cylinder with its axis along the z axis of its frame and its
center at the origin of the frame: of radius \f$r\f$, between \f$z = -h\f$ and \f$z = h\f$.
Given by its radius and half length (with the optional pose), or fitted to the
body's mesh: along the principal axis of greatest spread of the vertices,
enclosing them.

Against a plane its contact is a line along the lowest generator, see
ComputeManifold.
*/
class Cylinder : public Base {
public:
  Cylinder()
    : Base() {
    type_ = "cylinder";
    DeclareParameters();
  }
  Cylinder(std::string name)
    : Base(name) {
    type_ = "cylinder";
    DeclareParameters();
  }
  virtual ~Cylinder() override {}

  static std::unique_ptr<Cylinder> Create();

  static bool is_registered() {
    return registered_;
  }

  double get_radius() const {
    return radius_;
  }

  /*! \brief half of the length of the cylinder along its axis */
  double get_half_length() const {
    return half_length_;
  }

  /*! \brief the axis of the cylinder in the inertial frame, a unit vector,
   * valid after UpdatePose */
  const ksl_vec3_t& get_axis() const {
    return d_absolute_.R.v[2];
  }

  virtual void Support(const ksl_vec3_t& direction, ksl_vec3_t& local,
                       ksl_vec3_t& global, int& hint) const override;

protected:
  virtual void DeclareParameters() override;
  virtual bool HasDimensions() const override;
  virtual void SetFromDimensions(const ksl_SE3_t& pose) override;
  virtual void Fit(const std::vector<ksl_vec3_t>& vertices,
                   const std::vector<unsigned int>& faces) override;
  virtual void CopyFit(const Base& other) override;

private:
  static bool registered_;
  double radius_ = 0.0;
  double half_length_ = 0.0;
};

} // namespace collision_geometry
} // namespace core
} // namespace shift

#endif
