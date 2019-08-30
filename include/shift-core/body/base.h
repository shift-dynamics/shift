#ifndef SHIFT_BODY_BASE_H_
#define SHIFT_BODY_BASE_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/inertia.h"

#include "shift-core/base.h"
#include "shift-core/collisiongeometry/base.h"
#include "shift-core/frame.h"
#include "shift-core/geometry/base.h"
#include "shift-core/light/base.h"
#include "shift-core/parameter.h"

namespace Shift {

namespace Joint {
class Base;
}

class Assembly;

class Context;

namespace Body {

/*!
@brief Parameters that describe a Shift::Body
*/
class Base
  : public Shift::Base
  , public Shift::Parameter::List {

public:
  Base(std::string name)
    : Shift::Base(name)
    , Shift::Parameter::List() {
    Frame* origin = new Frame("origin");
    origin->setIsOrigin(true);
    origin->setIsReference(true);
    origin->setParentBody(this);
    appendFrame(origin);
    addParameters();
  }
  Base()
    : Shift::Base("") {
    Frame* origin = new Frame("origin");
    origin->setIsOrigin(true);
    origin->setIsReference(true);
    origin->setParentBody(this);
    appendFrame(origin);
    addParameters();
  }
  Base(Base&& b)
    : Shift::Base(b.name_) {
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
  virtual ~Base() {}

  virtual void addParameters();

  const std::string& type() {
    return type_;
  }

  void setAssembly(Assembly* assembly) {
    assembly_ = assembly;
  }
  void setContext(Context* context) {
    context_ = context;
  }
  void appendFrame(Frame*);
  // void appendLight(std::unique_ptr<Light::Base>&);
  // void appendGeometry(std::unique_ptr<Geometry::Base>&);
  // void appendCollisionGeometry(std::unique_ptr<CollisionGeometry::Base>&);

  Base* parentBody();
  virtual void load();
  virtual void connect();
  void setFullyQualifiedNames(std::vector<std::string>&);
  void setActiveObjects(bool);
  Frame* getFrame(std::string);
  Frame* origin();
  Frame* referenceFrame();
  void setReferenceFrame(Shift::Frame*);
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
    vaIndex_ = index;
  }

  int vaIndex() {
    return vaIndex_;
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
    return isVirtual_;
  }

  const ksl_vec3_t centerOfMassWRTReference();

protected:
  void setDWRTReference();

  std::string type_;
  bool is_articulated_ = true;   /*!< indicates whether body may have joints
                                   attached to it, i.e. that the body comprises a
                                   link of an articulated assembly of bodies */
  Assembly* assembly_ = nullptr; /*!< Pointer to assembly that this body is a
                                  member of. */
  Context* context_ = nullptr;

  Parameter::Array<Frame> frames_;

  Parameter::FactoryArray<Light::Base> lights_;
  Parameter::FactoryArray<Geometry::Base> geometry_;
  Parameter::FactoryArray<CollisionGeometry::Base> collision_geometry_;

  bool draw_enabled_ = true;
  bool draw_collision_geometry_enabled_ = false;
  int index_ = -1;
  int level_ = -1;
  int vaIndex_ = -1;
  Joint::Base* parent_joint_ = nullptr;
  Shift::Frame* reference_frame_ = nullptr;
  ksl_inertia_t inertia_ = {}; /*!< centroidal Mass and moment of Inertia
                                    properties expressed in body origin
                                    coordinate system */
  ksl_vec3_t center_of_mass_wrt_origin_ = {};    /*!< vector from body origin to
                                                center of mass */
  ksl_vec3_t center_of_mass_wrt_reference_ = {}; /*!< vector from body reference
                                                frame to center of mass */
  bool isVirtual_ = false;
};

class Modules {
public:
  static bool initialized();
};

} // namespace Body

} // namespace Shift

#endif
