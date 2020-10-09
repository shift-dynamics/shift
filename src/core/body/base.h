#ifndef SHIFT_BODY_BASE_H_
#define SHIFT_BODY_BASE_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/inertia.h"

#include "shift/core/base.h"
#include "shift/core/collisiongeometry/base.h"
#include "shift/core/frame.h"
#include "shift/core/geometry/base.h"
#include "shift/core/light/base.h"
#include "shift/core/parameter/array.h"
#include "shift/core/parameter/factoryarray.h"
#include "shift/core/parameter/list.h"

namespace shift {

namespace Joint {
class Base;
}

class Assembly;

class Context;

namespace Body {

/*!
@brief Parameters that describe a shift::Body
*/
class Base
  : public shift::Base
  , public shift::Parameter::List {

public:
  Base(std::string name)
    : shift::Base(name)
    , shift::Parameter::List() {
    auto origin = std::make_unique<Frame>("origin");
    origin->setIsOrigin(true);
    origin->setIsReference(true);
    origin->setParentBody(this);
    assert(origin->parentBody());

    frames_.push_back(std::move(origin));
    addParameters();
  }
  Base()
    : shift::Base("")
    , shift::Parameter::List() {
    auto origin = std::make_unique<Frame>("origin");
    origin->setIsOrigin(true);
    origin->setIsReference(true);
    origin->setParentBody(this);
    frames_.push_back(std::move(origin));
    addParameters();
  }
  Base(Base&& b)
    : shift::Base(b.name_) {
    // for(auto& i : b.lights_) {
    //   lights_.push_back(std::move(i));
    // }
    // for(auto& i : b.geometry_) {
    //   geometry_.push_back(std::move(i));
    // }
    // for(auto& i : b.collisionGeometry_) {
    //   collisionGeometry_.push_back(std::move(i));
    // }
  }
  virtual ~Base() override {}

  virtual void setActiveObjects(bool parent_is_active) override;

  const std::string& type() {
    return type_;
  }

  Base* parentBody();
  virtual void load();
  virtual void connect();

  Frame* origin();
  Frame* referenceFrame();
  void setReferenceFrame(shift::Frame*);
  void setReferenceFrame();
  unsigned int frameCount() {
    return frames_.size();
  }

  static const std::string arrayKey() {
    return "bodies";
  }

  static const std::string defaultType() {
    return "rigid";
  }

  /*!
  return number of frames in spanning tree, context->processFrames must be
  called to set frames in spanning tree.
  */
  unsigned int frameInTreeCount();

  int index() {
    return index_;
  }

  void setIndex(int index) {
    index_ = index;
  }

  int level() {
    return level_;
  }

  std::vector<std::unique_ptr<Frame>>& frames() {
    return frames_.data();
  }

  void setLevel(int level) {
    level_ = level;
  }

  const Joint::Base* parentJoint() {
    return parent_joint_;
  }

  void setParentJoint(Joint::Base*);

  bool isArticulated() {
    return is_articulated_;
  }

  void setvaIndex(int index) {
    va_index_ = index;
  }

  int vaIndex() {
    return va_index_;
  }

  void setDWRTReference(ksl_SE3_t);

  void clearInTreeFlags();

  virtual const ksl_inertia_t& inertia() const {
    return inertia_;
  }

  virtual ksl_inertia_t inertiaWRTReference() {
    return inertia_;
  }

  bool hasMass() {
    if(inertia_.m >= 1e-9) {
      return true;
    }
    return false;
  }

  bool isVirtual() {
    return is_virtual_;
  }

  const ksl_vec3_t centerOfMassWRTReference();

protected:
  virtual void addParameters() override;
  void setDWRTReference();

  std::string type_;
  bool is_articulated_ = true; /*!< indicates whether body may have joints
                                 attached to it, i.e. that the body comprises a
                                 link of an articulated assembly of bodies */

  Parameter::Array<Frame> frames_;

  Parameter::FactoryArray<Light::Base> lights_;
  Parameter::FactoryArray<Geometry::Base> geometry_;
  Parameter::FactoryArray<CollisionGeometry::Base> collision_geometry_;

  bool draw_enabled_ = true;
  bool draw_collision_geometry_enabled_ = false;
  int index_ = -1;
  int level_ = -1;
  int va_index_ = -1;
  Joint::Base* parent_joint_ = nullptr;
  shift::Frame* reference_frame_ =
    nullptr; /*!< pointer to unique reference frame on body, in an articulated
                multibody chain, this is the *proximal* or upstream
                frame in the spanning tree */
  ksl_inertia_t inertia_ = {}; /*!< centroidal mass and moment of inertia
                                    properties expressed in the body's origin
                                    coordinate system */
  ksl_vec3_t center_of_mass_wrt_origin_ = {};    /*!< vector from body origin to
                                                center of mass */
  ksl_vec3_t center_of_mass_wrt_reference_ = {}; /*!< vector from body reference
                                                frame to center of mass */
  bool is_virtual_ = false;
};

class Modules {
public:
  static bool initialized();
};

} // namespace Body

} // namespace shift

#endif
