#ifndef SHIFT_COLLISIONGEOMETRY_CAPSULE_H_
#define SHIFT_COLLISIONGEOMETRY_CAPSULE_H_

#include <memory>

#include "shift/core/collision_geometry/base.h"

namespace shift {
namespace core {
namespace collision_geometry {

/*!
@brief capsule collision geometry

A cylinder with a hemisphere on each end: the points within the radius r of
the segment on the z axis of its frame between z = -h and z = h, centered at
the origin of the frame. Given by its radius and the half length of the
segment (with the optional pose), or fitted to the body's mesh: along the
principal axis of greatest spread of the vertices, enclosing them.

Against a plane its contact is a line along the segment, see
ComputeManifold.
*/
class Capsule : public Base {
public:
  Capsule()
    : Base() {
    type_ = "capsule";
    DeclareParameters();
  }
  Capsule(std::string name)
    : Base(name) {
    type_ = "capsule";
    DeclareParameters();
  }
  virtual ~Capsule() override {}

  static std::unique_ptr<Capsule> Create();

  static bool is_registered() {
    return registered_;
  }

  double get_radius() const {
    return radius_;
  }

  /*! \brief half of the length of the segment of the capsule, without its
   * end caps */
  double get_half_length() const {
    return half_length_;
  }

  /*! \brief the axis of the capsule in the inertial frame, a unit vector,
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
