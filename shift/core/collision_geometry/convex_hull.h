#ifndef SHIFT_COLLISIONGEOMETRY_CONVEXHULL_H_
#define SHIFT_COLLISIONGEOMETRY_CONVEXHULL_H_

#include <memory>

#include "shift/core/collision_geometry/base.h"
#include "shift/core/collision_geometry/fitting.h"

namespace shift {
namespace core {
namespace collision_geometry {

/*!
@brief convex hull collision geometry

A single convex hull, computed with qhull, encompassing all mesh geometry of
the body. The reference frame is the body origin. The extreme vertex support
function climbs the hull's vertex adjacency graph from the previous extreme
vertex, which is usually close to the new one.
*/
class ConvexHull : public Base {
public:
  ConvexHull()
    : Base() {
    type_ = "convex_hull";
    DeclareParameters();
  }
  ConvexHull(std::string name)
    : Base(name) {
    type_ = "convex_hull";
    DeclareParameters();
  }
  virtual ~ConvexHull() override {}

  static std::unique_ptr<ConvexHull> Create();

  static bool is_registered() {
    return registered_;
  }

  /*! \brief hull vertices, faces and adjacency, in the body origin frame */
  const ConvexHullData& get_hull() const {
    return *hull_;
  }

  virtual void Support(const ksl_vec3_t& direction, ksl_vec3_t& local,
                       ksl_vec3_t& global, int& hint) const override;

  virtual std::shared_ptr<const CadContactShape>
  MakeExactShape() const override;
  virtual const void* ExactShapeKey() const override {
    return hull_.get();
  }
  virtual double get_exact_edge_radius() const override {
    return edge_radius_;
  }

protected:
  virtual void DeclareParameters() override;
  virtual bool HasDimensions() const override;
  virtual void SetFromDimensions(const ksl_SE3_t& pose) override;
  virtual void Fit(const std::vector<ksl_vec3_t>& vertices,
                   const std::vector<unsigned int>& faces) override;
  virtual void CopyFit(const Base& other) override;

  /*! \brief set the hull from its vertices and faces (the part of Fit that
   * does not depend on the type, so a box can fit its own way) */
  void SetHull(const std::vector<ksl_vec3_t>& vertices,
               const std::vector<unsigned int>& faces);

  ksl_vec3_t half_extents_ = {{0.0, 0.0, 0.0}}; //!< of a box, if given
  double edge_radius_ = 0.0;

private:
  static bool registered_;
  std::shared_ptr<const ConvexHullData> hull_ =
    std::make_shared<ConvexHullData>();
};

} // namespace collision_geometry
} // namespace core
} // namespace shift

#endif
