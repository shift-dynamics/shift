#ifndef _SHIFT_BODY_H_
#define _SHIFT_BODY_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/inertia.h"

#include "shift-core/base.h"
#include "shift-core/collisiongeometry/collisiongeometry.h"
#include "shift-core/frame.h"
#include "shift-core/geometry/geometry.h"
#include "shift-core/lights/light.h"
#include "shift-core/serialize.h"

namespace Shift {

namespace Joint {
class JointBase;
}

class Assembly;

class Context;

namespace Body {

/*!
@brief Parameters that describe a Shift::Body
*/
class BodyBase
  : public Base
  , public Serialize {

public:
  BodyBase(std::string name)
    : Base(name) {
    Frame origin("origin");
    appendFrame(origin);
  }
  BodyBase()
    : Base("") {
    Frame origin("origin");
    appendFrame(origin);
  }
  BodyBase(BodyBase&& b)
    : Base(b.name_) {
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
  ~BodyBase() {}

  virtual void addKeys();

  const std::string& getType() {
    return type_;
  }

  void setAssembly(Assembly* assembly) {
    assembly_ = assembly;
  }
  void setContext(Context* context) {
    context_ = context;
  }
  void appendFrame(Frame&);
  void appendLight(std::unique_ptr<Light::LightBase>&);
  void appendGeometry(std::unique_ptr<Geometry::GeometryBase>&);
  void appendCollisionGeometry(
    std::unique_ptr<CollisionGeometry::CollisionGeometryBase>&);
  Joint::JointBase* getParentJoint();
  BodyBase* getParentBody();
  virtual void readJSON(Json::Value&);
  virtual void print();
  virtual void load();
  virtual void connect();
  void setFullyQualifiedNames(std::vector<std::string>&);
  void setActiveObjects(bool);

protected:
  std::string type_;
  Assembly* assembly_ = NULL; /*!< Pointer to assembly that this body is a
                                  member of. */
  Context* context_ = NULL;
  std::vector<Frame> frames_; /*!< Frame list */
  std::vector<std::unique_ptr<Light::LightBase>> lights_;
  std::vector<std::unique_ptr<Geometry::GeometryBase>> geometry_;
  std::vector<std::unique_ptr<CollisionGeometry::CollisionGeometryBase>>
    collisionGeometry_;
  bool drawEnabled_ = true;
  bool drawCollisionObjectEnabled_ = false;
};

class Modules {
public:
  static bool initialized();
};

} // namespace Body

} // namespace Shift

#endif
