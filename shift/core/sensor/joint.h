#ifndef SHIFT_SENSOR_JOINT_H_
#define SHIFT_SENSOR_JOINT_H_

#include <memory>
#include <string>

#include "shift/core/sensor/base.h"

namespace shift {
namespace core {

namespace joint {
class Base;
}

namespace sensor {

/*!
@brief Measures a generalized coordinate or velocity of a joint

JSON:
\code
{"type": "joint", "name": "rack_position", "joint": "L",
 "quantity": "position", "index": 0}
\endcode
quantity is "position" (the coordinate, with its velocity as rate) or
"velocity"; index selects the degree of freedom of a multi-DOF joint.
Positions are available for joints whose coordinates are not quaternions.
*/
class Joint : public Base {
public:
  Joint()
    : Base() {
    DeclareParameters();
  }

  Joint(std::string name)
    : Base(name) {
    DeclareParameters();
  }

  virtual ~Joint() override {}

  static std::unique_ptr<Joint> Create();

  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Update(const double* y) override;

protected:
  void DeclareParameters() override;

private:
  static bool registered_;
  std::string joint_name_;
  std::string quantity_;
  int index_ = 0;
  joint::Base* joint_ = nullptr;
  bool position_ = true;
};

} // namespace sensor
} // namespace core
} // namespace shift

#endif
