#ifndef _SHIFT_BODY_H_
#define _SHIFT_BODY_H_

#include <string>
#include <vector>

#include "ksl/inertia.h"

#include "shift-core/base.h"
#include "shift-core/frame.h"
#include "shift-core/serialize.h"

namespace Shift {

namespace Joint {
class JointBase;
}

namespace Light {
class LightBase;
}

namespace Geometry {
class GeometryBase;
}

namespace CollisionGeometry {
class CollisionGeometryBase;
}

class Assembly;

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
  ~BodyBase() {}

  virtual void addKeys();

  const std::string& getType() {
    return type_;
  }

  void setParentAssembly(Assembly*);
  void appendFrame(Frame&);
  void appendLight(Light::LightBase*);
  void appendGeometry(Geometry::GeometryBase*);
  void appendCollisionGeometry(CollisionGeometry::CollisionGeometryBase*);
  Joint::JointBase* getParentJoint();
  BodyBase* getParentBody();
  virtual void readJSON(Json::Value&);
  virtual void print();

protected:
  std::string type_;
  Assembly* assembly_ = NULL; /*!< Pointer to assembly that this body is a
                                  member of. */

  std::vector<Frame> frames_; /*!< Frame list */
  std::vector<Light::LightBase*> lights_;
  std::vector<Geometry::GeometryBase*> geometry_;
  std::vector<CollisionGeometry::CollisionGeometryBase*> collisionGeometry_;

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
