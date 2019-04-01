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
#include "shift-core/serialize.h"

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
  , public Serialize {

public:
  Base(std::string name)
    : Shift::Base(name) {
    Frame origin("origin");
    origin.setIsOrigin(true);
    origin.setIsReference(true);
    origin.setParentBody(this);
    appendFrame(origin);
  }
  Base()
    : Shift::Base("") {
    Frame origin("origin");
    origin.setIsOrigin(true);
    origin.setIsReference(true);
    origin.setParentBody(this);
    appendFrame(origin);
  }
  Base(Base&& b)
    : Shift::Base(b.name_) {
    for(auto& i : b.lights_) {
      lights_.push_back(std::move(i));
    }
    for(auto& i : b.geometry_) {
      geometry_.push_back(std::move(i));
    }
    for(auto& i : b.collisionGeometry_) {
      collisionGeometry_.push_back(std::move(i));
    }
  }
  virtual ~Base() {}

  virtual void addKeys();

  const std::string& type() {
    return type_;
  }

  void setAssembly(Assembly* assembly) {
    assembly_ = assembly;
  }
  void setContext(Context* context) {
    context_ = context;
  }
  void appendFrame(Frame&);
  void appendLight(std::unique_ptr<Light::Base>&);
  void appendGeometry(std::unique_ptr<Geometry::Base>&);
  void appendCollisionGeometry(std::unique_ptr<CollisionGeometry::Base>&);

  Base* parentBody();
  virtual void readJSON(Json::Value&);
  virtual void print();
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

  std::vector<Shift::Frame>& frames() {
    return frames_;
  }

  void setLevel(int level) {
    level_ = level;
  }

  Joint::Base* parentJoint() {
    return parentJoint_;
  }

  void setParentJoint(Joint::Base*);

  bool isArticulated() {
    return isArticulated_;
  }

  void setVAIndex(int index) {
    vaIndex_ = index;
  }

  int VAIndex() {
    return vaIndex_;
  }

  void setDWRTReference(ksl_SE3_t);

  void clearInTreeFlags();

protected:
  void setDWRTReference();

  std::string type_;
  bool isArticulated_ = true; /*!< indicates whether body may have joints
                                 attached to it, i.e. that the body comprises a
                                 link of an articulated assembly of bodies */
  Assembly* assembly_ = NULL; /*!< Pointer to assembly that this body is a
                                  member of. */
  Context* context_ = NULL;
  std::vector<Frame> frames_; /*!< Frame list */
  std::vector<std::unique_ptr<Light::Base>> lights_;
  std::vector<std::unique_ptr<Geometry::Base>> geometry_;
  std::vector<std::unique_ptr<CollisionGeometry::Base>> collisionGeometry_;
  bool drawEnabled_ = true;
  bool drawCollisionObjectEnabled_ = false;
  int index_ = -1;
  int level_ = -1;
  int vaIndex_ = -1;
  Joint::Base* parentJoint_ = NULL;
  Shift::Frame* referenceFrame_ = NULL;
};

class Modules {
public:
  static bool initialized();
};

} // namespace Body

} // namespace Shift

#endif
