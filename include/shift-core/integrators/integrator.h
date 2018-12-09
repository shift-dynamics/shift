#ifndef _SHIFT_INTEGRATOR_H_
#define _SHIFT_INTEGRATOR_H_

#include <functional>
#include <map>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

class Frame;

namespace Integrator {

/*!
@brief Abstract Integrator base class
*/
class IntegratorBase
  : public Base
  , public Serialize {

public:
  IntegratorBase(std::string name)
    : Base(name) {
    addKeys();
  }
  IntegratorBase()
    : Base("") {
    addKeys();
  }
  ~IntegratorBase() {}

  const std::string& getType() {
    return type_;
  }

  void readJSON(Json::Value);

protected:
  void addKeys();

  std::string type_; //!< descriptive name of Integrator type
};

class Modules {
public:
  void list();
  static bool initialized();

private:
  static std::vector<std::string> modules_;
  static bool initialized_;
};

} // namespace Integrator

} // namespace Shift

#endif
