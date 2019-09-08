#ifndef SHIFT_ASSEMBLY_H_
#define SHIFT_ASSEMBLY_H_

#include <string>

#include "shift-core/base.h"
#include "shift-core/behavior/base.h"
#include "shift-core/body/base.h"
#include "shift-core/constraint/base.h"
#include "shift-core/forcemodule/base.h"
#include "shift-core/joint/base.h"
#include "shift-core/light/base.h"
#include "shift-core/parameter.h"

namespace Shift {

namespace Behavior {
class Base;
}
namespace Body {
class Base;
}
namespace Constraint {
class Base;
}
namespace ForceModule {
class Base;
}
namespace Joint {
class Base;
}
namespace Light {
class Base;
}

class Context;
class SerialEntry;

/*!
@brief Shift Assembly datastructure
*/
class Assembly
  : public Base
  , public Parameter::List {

public:
  Assembly(std::string name)
    : Base(name)
    , Parameter::List() {
    addParameters();
  }

  Assembly()
    : Base("")
    , Parameter::List() {
    addParameters();
  }

  virtual ~Assembly() override {}

  static const std::string arrayKey() {
    return "assemblies";
  }

private:
  void addParameters() override;

  Shift::Parameter::FactoryArray<Body::Base> bodies_;
  Shift::Parameter::FactoryArray<Behavior::Base> behaviors_;
  Shift::Parameter::FactoryArray<Constraint::Base> constraints_;
  Shift::Parameter::FactoryArray<ForceModule::Base> force_modules_;
  Shift::Parameter::FactoryArray<Joint::Base> joints_;

  Shift::Parameter::Array<Assembly> assemblies_;
};

} // namespace Shift

#endif
