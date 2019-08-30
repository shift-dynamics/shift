#ifndef SHIFT_ASSEMBLY_H_
#define SHIFT_ASSEMBLY_H_

#include <json/json.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

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

  ~Assembly() {}

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
  Body::Base* getBody(unsigned int i) {
    return bodies_.get(i);
  }
  static const std::string arrayKey() {
    return "assemblies";
  }

private:
  Context* context_ = nullptr;
  void addParameters() override;

  Shift::Parameter::Array<Assembly> assemblies_;

  Shift::Parameter::FactoryArray<Body::Base> bodies_;
  Shift::Parameter::FactoryArray<Behavior::Base> behaviors_;
  Shift::Parameter::FactoryArray<Constraint::Base> constraints_;
  Shift::Parameter::FactoryArray<ForceModule::Base> force_modules_;
  Shift::Parameter::FactoryArray<Joint::Base> joints_;
};

} // namespace Shift

#endif
