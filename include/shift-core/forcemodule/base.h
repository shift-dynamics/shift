#ifndef SHIFT_FORCEMODULE_BASE_H_
#define SHIFT_FORCEMODULE_BASE_H_

#include <functional>
#include <map>

#include "shift-core/base.h"
#include "shift-core/parameter.h"

namespace Shift {

class Context;

namespace ForceModule {

/*!
@brief Shift::Frame abstract base class
*/
class Base
  : public Shift::Base
  , public Parameter::List {

public:
  Base(std::string name)
    : Shift::Base(name)
    , Parameter::List() {
    addParameters();
  }
  Base()
    : Shift::Base("")
    , Parameter::List() {
    addParameters();
  }
  virtual ~Base() {}

  static const std::string arrayKey() {
    return "force_modules";
  }
  static const std::string defaultType() {
    return "";
  }
  virtual void load();
  virtual void connect();
  virtual void setActiveObjects(bool parent_is_active) override;

protected:
  virtual void addParameters() override;

  std::string type_;
};

class Modules {
public:
  static bool initialized();
};

} // namespace ForceModule

} // namespace Shift

#endif
