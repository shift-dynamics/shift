#ifndef SHIFT_COLLISIONGEOMETRY_SPHERE_H_
#define SHIFT_COLLISIONGEOMETRY_SPHERE_H_

#include <memory>

#include "shift/core/collision_geometry/base.h"

namespace shift {
namespace core {
namespace collision_geometry {

/*!
@brief bounding sphere collision geometry

Fitted to the convex hull of the body's mesh with Ritter's algorithm, starting
from the sphere spanning the extreme points along the principal axis of the
vertex covariance. The result is approximately 5% larger than the minimum
bounding sphere.

Ritter, J. (1990), "An efficient bounding sphere", in Graphics Gems,
A. Glassner, Ed. Academic Press, pp. 301-303
*/
class Sphere : public Base {
public:
  Sphere()
    : Base() {
    type_ = "sphere";
    DeclareParameters();
  }
  Sphere(std::string name)
    : Base(name) {
    type_ = "sphere";
    DeclareParameters();
  }
  virtual ~Sphere() override {}

  static std::unique_ptr<Sphere> Create();

  static bool is_registered() {
    return registered_;
  }

  double get_radius() const {
    return radius_;
  }

  /*! \brief absolute position of the sphere center, valid after UpdatePose */
  const ksl_vec3_t& get_center() const {
    return d_absolute_.t;
  }

  virtual void Support(const ksl_vec3_t& direction, ksl_vec3_t& local,
                       ksl_vec3_t& global, int& hint) const override;

  /*! \brief in closed form, see Base::RayCast */
  bool RayCast(const ksl_vec3_t& origin, const ksl_vec3_t& direction,
               double max_distance, double& distance,
               ksl_vec3_t& normal) const override;

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
};

} // namespace collision_geometry
} // namespace core
} // namespace shift

#endif
