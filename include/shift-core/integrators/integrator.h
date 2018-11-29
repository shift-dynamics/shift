#ifndef _SHIFT_INTEGRATOR_H_
#define _SHIFT_INTEGRATOR_H_

#include <string>

#include "shift-core/base.h"

namespace Shift {

/*!
@brief Abstract base class for Shift Integrator C++ objects
*/
class Integrator : Base {

public:
  Integrator(std::string name)
    : Base(name) {}
  ~Integrator() {}

  const std::string& getType() {
    return type_;
  }

private:
  std::string type_;
};

} // namespace Shift

#endif
