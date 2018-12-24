#ifndef _SHIFT_ASSEMBLY_H_
#define _SHIFT_ASSEMBLY_H_

#include <json/json.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "shift-core/base.h"
#include "shift-core/behaviors/behavior.h"
#include "shift-core/bodies/body.h"
#include "shift-core/constraints/constraint.h"
#include "shift-core/forcemodules/forcemodule.h"
#include "shift-core/joints/joint.h"
#include "shift-core/lights/light.h"
#include "shift-core/serialize.h"

namespace Shift {

namespace Behavior {
class BehaviorBase;
}
namespace Body {
class BodyBase;
}
namespace Constraint {
class ConstraintBase;
}
namespace ForceModule {
class ForceModuleBase;
}
namespace Joint {
class JointBase;
}
namespace Light {
class LightBase;
}

class Context;
class SerialEntry;

/*!
@brief Shift Assembly datastructure
*/
class Assembly
  : public Base
  , public Serialize {

public:
  Assembly(std::string name)
    : Base(name)
    , Serialize() {
    addKeys();
  }

  Assembly()
    : Base("")
    , Serialize() {
    addKeys();
  }

  Assembly(Assembly&& a)
    : Base(a.name_) {
    for(auto& i : a.behaviors_) {
      behaviors_.push_back(std::move(i));
    }
    for(auto& i : a.bodies_) {
      bodies_.push_back(std::move(i));
    }
    for(auto& i : a.constraints_) {
      constraints_.push_back(std::move(i));
    }
    for(auto& i : a.forcemodules_) {
      forcemodules_.push_back(std::move(i));
    }
    for(auto& i : a.joints_) {
      joints_.push_back(std::move(i));
    }
  }

  Assembly& operator=(Assembly& a) {
    // check for self-assignment
    if(&a == this) {
      return *this;
    }

    for(auto& i : a.behaviors_) {
      behaviors_.push_back(std::move(i));
    }
    for(auto& i : a.bodies_) {
      bodies_.push_back(std::move(i));
    }
    for(auto& i : a.constraints_) {
      constraints_.push_back(std::move(i));
    }
    for(auto& i : a.forcemodules_) {
      forcemodules_.push_back(std::move(i));
    }
    for(auto& i : a.joints_) {
      joints_.push_back(std::move(i));
    }
    return *this;
  }

  ~Assembly() {}

  void readJSON(Json::Value& value, bool fixed = false);
  void printBehaviors();
  void printBodies();
  void printConstraints();
  void printForceModules();
  void printJoints();
  void print();
  void setFullyQualifiedNames(std::vector<std::string>&);
  void setActiveObjects(bool);
  void setContext(Context* context) {
    context_ = context;
  }

private:
  Context* context_;
  void addKeys();

  std::vector<std::unique_ptr<Assembly>> subassemblies_;

  std::vector<std::unique_ptr<Behavior::BehaviorBase>> behaviors_;
  std::vector<std::unique_ptr<Body::BodyBase>> bodies_;
  std::vector<std::unique_ptr<Constraint::ConstraintBase>> constraints_;
  std::vector<std::unique_ptr<ForceModule::ForceModuleBase>> forcemodules_;
  std::vector<std::unique_ptr<Joint::JointBase>> joints_;
};

} // namespace Shift

#endif
