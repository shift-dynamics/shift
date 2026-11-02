#ifndef SHIFT_COLLISIONGEOMETRY_ELLIPSOID_H_
#define SHIFT_COLLISIONGEOMETRY_ELLIPSOID_H_

#include <memory>

#include "shift/core/collision_geometry/base.h"

namespace shift {
namespace core {
namespace collision_geometry {

/*!
@brief minimum volume bounding ellipsoid collision geometry

Fitted to the convex hull of the body's mesh with Khachiyan's algorithm, as
described in:

Moshtagh, Nima. "Minimum volume enclosing ellipsoid." Convex Optimization 111
(2005): 112.

The collision geometry reference frame is located at the ellipsoid center with
its axes along the principal axes of the ellipsoid, with semi-axis lengths a, b
and c along x, y and z.
*/
class Ellipsoid : public Base {
public:
  Ellipsoid()
    : Base() {
    type_ = "ellipsoid";
    DeclareParameters();
  }
  Ellipsoid(std::string name)
    : Base(name) {
    type_ = "ellipsoid";
    DeclareParameters();
  }
  virtual ~Ellipsoid() override {}

  static std::unique_ptr<Ellipsoid> Create();

  static bool is_registered() {
    return registered_;
  }

  /*! \brief semi-axis lengths along the x, y and z axes of the collision
   * geometry reference frame */
  const ksl_vec3_t& get_semi_axes() const {
    return semi_axes_;
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
  double tolerance_ = 0.001;
  ksl_vec3_t semi_axes_ = {{0.0, 0.0, 0.0}};
};

} // namespace collision_geometry
} // namespace core
} // namespace shift

#endif
