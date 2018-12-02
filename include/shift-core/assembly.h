#ifndef _SHIFT_ASSEMBLY_H_
#define _SHIFT_ASSEMBLY_H_

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

/*!
@brief Shift Assembly datastructure
*/
class Assembly
  : public Base
  , public Serialize {

public:
  Assembly(std::string name)
    : Base(name)
    , Serialize() {}

  Assembly()
    : Base("")
    , Serialize() {}

  ~Assembly() {}

private:
  void addKeys();

  std::vector<Assembly> subassemblies_;

  std::vector<Behavior::BehaviorBase*> behaviors_;
  std::vector<Body::BodyBase*> bodies_;
  std::vector<Constraint::ConstraintBase*> constraints_;
  std::vector<ForceModule::ForceModuleBase*> forcemodules_;
  std::vector<Joint::JointBase*> joints_;
  std::vector<Light::LightBase*> lights_;
};

} // namespace Shift

#endif
