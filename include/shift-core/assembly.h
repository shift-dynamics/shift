#ifndef _SHIFT_ASSEMBLY_H_
#define _SHIFT_ASSEMBLY_H_

#include <json/json.h>
#include <map>
#include <string>
#include <vector>

#include "shift-core/base.h"
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

  ~Assembly() {}

  void readJSON(Json::Value& value, bool fixed = false);
  void printBehaviors();
  void printBodies();
  void printConstraints();
  void printForceModules();
  void printJoints();
  void print();

private:
  void addKeys();

  std::vector<Assembly> subassemblies_;

  std::vector<Behavior::BehaviorBase*> behaviors_;
  std::vector<Body::BodyBase*> bodies_;
  std::vector<Constraint::ConstraintBase*> constraints_;
  std::vector<ForceModule::ForceModuleBase*> forcemodules_;
  std::vector<Joint::JointBase*> joints_;
};

} // namespace Shift

#endif
