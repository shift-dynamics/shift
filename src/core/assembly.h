#ifndef SHIFT_ASSEMBLY_H_
#define SHIFT_ASSEMBLY_H_

#include <string>

#include "shift/core/base.h"
#include "shift/core/behavior/base.h"
#include "shift/core/body/base.h"
#include "shift/core/constraint/base.h"
#include "shift/core/forcemodule/base.h"
#include "shift/core/joint/base.h"
#include "shift/core/light/base.h"
#include "shift/core/parameter/list.h"
#include "shift/core/sensor/base.h"

namespace shift {

class Context;

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

  shift::Parameter::FactoryArray<Body::Base> bodies_;
  shift::Parameter::FactoryArray<Behavior::Base> behaviors_;
  shift::Parameter::FactoryArray<Constraint::Base> constraints_;
  shift::Parameter::FactoryArray<ForceModule::Base> force_modules_;
  shift::Parameter::FactoryArray<Joint::Base> joints_;
  shift::Parameter::FactoryArray<Sensor::Base> sensors_;

  shift::Parameter::Array<Assembly> assemblies_;
};

} // namespace shift

#endif
