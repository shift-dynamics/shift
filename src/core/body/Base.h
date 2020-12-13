#ifndef SHIFT_BODY_BASE_H_
#define SHIFT_BODY_BASE_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/inertia.h"

#include "shift/core/Base.h"
#include "shift/core/collisiongeometry/Base.h"
#include "shift/core/Frame.h"
#include "shift/core/geometry/Base.h"
#include "shift/core/light/Base.h"
#include "shift/core/parameter/Array.h"
#include "shift/core/parameter/FactoryArray.h"
#include "shift/core/parameter/List.h"

namespace shift {
namespace core {

namespace joint {
class Base;
}

class Assembly;
class Context;

namespace body {

/*!
@brief Parameters that describe a shift::Body
*/
class Base
  : public shift::core::Base
  , public shift::core::parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name)
    , shift::core::parameter::List() {
    auto origin = std::make_unique<Frame>("origin");
    origin->setIsOrigin(true);
    origin->setIsReference(true);
    origin->setParentBody(this);
    assert(origin->parentBody());

    frames_.push_back(std::move(origin));
    addParameters();
  }
  Base()
    : shift::core::Base("")
    , shift::core::parameter::List() {
    auto origin = std::make_unique<Frame>("origin");
    origin->setIsOrigin(true);
    origin->setIsReference(true);
    origin->setParentBody(this);
    frames_.push_back(std::move(origin));
    addParameters();
  }
  Base(Base&& b)
    : shift::core::Base(b.name_) {
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
  void setReferenceFrame(shift::core::Frame*);
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

  const joint::Base* parentJoint() {
    return parent_joint_;
  }

  void setParentJoint(joint::Base*);

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

  shift::core::parameter::Array<shift::core::Frame> frames_;

  shift::core::parameter::FactoryArray<shift::core::light::Base> lights_;
  shift::core::parameter::FactoryArray<shift::core::geometry::Base> geometry_;
  shift::core::parameter::FactoryArray<shift::core::collisiongeometry::Base> collision_geometry_;

  bool draw_enabled_ = true;
  bool draw_collision_geometry_enabled_ = false;
  int index_ = -1;
  int level_ = -1;
  int va_index_ = -1;
  joint::Base* parent_joint_ = nullptr;
  shift::core::Frame* reference_frame_ =
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

} // namespace body
}
} // namespace shift

#endif
