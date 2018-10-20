#ifndef _SHIFT_SENSOR_H_
#define _SHIFT_SENSOR_H_

#include <string>

#include "shift-core/base.h"

namespace Shift {

/*!
@brief Shift Assembly datastructure
*/
class Sensor : Base {

public:
  Sensor(std::string name)
    : Base(name) {}
  ~Sensor() {}

private:
};

} // namespace Shift

#endif
