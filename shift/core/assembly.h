#ifndef SHIFT_ASSEMBLY_H_
#define SHIFT_ASSEMBLY_H_

#include <string>

#include "shift/core/base.h"
#include "shift/core/behavior/base.h"
#include "shift/core/body/base.h"
#include "shift/core/constraint/base.h"
#include "shift/core/force_module/base.h"
#include "shift/core/joint/base.h"
#include "shift/core/light/base.h"
#include "shift/core/parameter/list.h"
#include "shift/core/sensor/base.h"

namespace shift {
namespace core {

class Context;

/*!
@brief Shift Assembly datastructure
*/
class Assembly
  : public Base
  , public parameter::List {

public:
  Assembly(std::string name)
    : Base(name)
    , parameter::List() {
    DeclareParameters();
  }

  Assembly()
    : Base("")
    , parameter::List() {
    DeclareParameters();
  }

  virtual ~Assembly() override {}

  static const std::string get_array_key() {
    return "assemblies";
  }

private:
  void DeclareParameters() override;

  shift::core::parameter::FactoryArray<body::Base> bodies_;
  shift::core::parameter::FactoryArray<behavior::Base> behaviors_;
  shift::core::parameter::FactoryArray<constraint::Base> constraints_;
  shift::core::parameter::FactoryArray<force_module::Base> force_modules_;
  shift::core::parameter::FactoryArray<joint::Base> joints_;
  shift::core::parameter::FactoryArray<sensor::Base> sensors_;

  shift::core::parameter::Array<Assembly> assemblies_;
};

}
} // namespace shift

#endif
