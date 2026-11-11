#ifndef SHIFT_COLLISIONGEOMETRY_BOX_H_
#define SHIFT_COLLISIONGEOMETRY_BOX_H_

#include <memory>

#include "shift/core/collision_geometry/convex_hull.h"

namespace shift {
namespace core {
namespace collision_geometry {

/*!
@brief box collision geometry, a convex hull of eight corners

A box is a convex hull, so it takes part in contact as one: its faces rest on
a plane or on the faces of other hulls through a manifold of points, each a
linear spring (see force_module::Contact). It is given by its half extents
along the axes of its pose, centered at the origin of the pose, or fitted to
the body's mesh: the oriented bounding box along the principal axes of the
vertices.
*/
class Box : public ConvexHull {
public:
  Box()
    : ConvexHull() {
    type_ = "box";
  }
  Box(std::string name)
    : ConvexHull(name) {
    type_ = "box";
  }
  virtual ~Box() override {}

  static std::unique_ptr<Box> Create();

  static bool is_registered() {
    return registered_;
  }

  /*! \brief half extents of the box along the axes of its frame */
  const ksl_vec3_t& get_half_extents() const {
    return half_extents_;
  }

  /*! \brief in closed form, see Base::RayCast */
  bool RayCast(const ksl_vec3_t& origin, const ksl_vec3_t& direction,
               double max_distance, double& distance,
               ksl_vec3_t& normal) const override;

protected:
  virtual bool HasDimensions() const override;
  virtual void Fit(const std::vector<ksl_vec3_t>& vertices,
                   const std::vector<unsigned int>& faces) override;
  virtual void CopyFit(const Base& other) override;

private:
  static bool registered_;
};

} // namespace collision_geometry
} // namespace core
} // namespace shift

#endif
