#ifndef _SHIFT_BODY_H_
#define _SHIFT_BODY_H_

#include <string>
#include <vector>

#include "ksl/inertia.h"

#include "shift-core/base.h"
#include "shift-core/frame.h"
#include "shift-core/geometry/geometry.h"
#include "shift-core/serialize.h"

namespace Shift {

namespace Joint {
class JointBase;
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
    addKeys();
  }
  BodyBase()
    : Base("") {
    addKeys();
  }
  ~BodyBase() {}

  void addKeys();

  void setParentAssembly(Assembly*);
  void appendFrame(Frame);
  Joint::JointBase* getParentJoint();
  BodyBase* getParentBody();

protected:
  Assembly* assembly_ = NULL; /*!< Pointer to assembly that this body is a
                                  member of. */

  std::vector<Frame> frames_; /*!< Frame list */
  std::vector<Geometry::GeometryBase*> geometry_;
  bool drawEnabled_ = true;
  bool drawCollisionObjectEnabled_ = false;
};

class Factory {

public:
  Factory();
  Factory(const Factory&) {}
  Factory& operator=(const Factory&) {
    return *this;
  }

  ~Factory() {
    factoryMap_.clear();
  }

  typedef std::function<BodyBase*()> CreateFunction;

  static Factory* get() {
    static Factory instance;
    return &instance;
  }

  static void registerModule(const std::string& name, CreateFunction pfnCreate);
  static BodyBase* create(const std::string& name);

private:
  static std::map<std::string, CreateFunction> factoryMap_;
};

class Registration {
public:
  Registration(std::string name, Factory::CreateFunction fcn) {
    Factory::registerModule(name, fcn);
  }
};

} // namespace Body

} // namespace Shift

#endif
