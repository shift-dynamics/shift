#ifndef SHIFT_SENSOR_FRAME_H_
#define SHIFT_SENSOR_FRAME_H_

#include <memory>
#include <string>
#include <vector>

#include "shift/core/sensor/base.h"

namespace shift {
namespace core {

class Frame;
namespace body {
class Base;
}

namespace sensor {

/*!
@brief Measures one component of the position or velocity of a frame

JSON:
\code
{"type": "frame", "name": "chassis_speed",
 "body_frame_pair": ["chassis", "origin"], "quantity": "velocity",
 "coordinate_system": "local", "dof": "Tx"}
\endcode
- quantity: "velocity" (Tx, Ty, Tz: linear velocity of the frame origin;
  Rx, Ry, Rz: angular velocity) or "position" (Tx, Ty, Tz of the frame
  origin, with the linear velocity as rate; global coordinates only)
- coordinate_system: "global" (fixed frame, the default) or "local" (axes of
  the measured frame)
*/
class Frame : public Base {
public:
  Frame()
    : Base() {
    DeclareParameters();
  }

  Frame(std::string name)
    : Base(name) {
    DeclareParameters();
  }

  virtual ~Frame() override {}

  static std::unique_ptr<Frame> Create();

  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Update(const double* y) override;

protected:
  void DeclareParameters() override;

private:
  static bool registered_;
  std::vector<std::string> body_frame_pair_;
  std::string quantity_;
  std::string coordinate_system_ = "global";
  std::string dof_;

  shift::core::body::Base* body_ = nullptr;
  shift::core::Frame* frame_ = nullptr;
  int component_ = 0; //!< 0..2: Tx..Tz, 3..5: Rx..Rz
  bool position_ = false;
  bool local_ = false;
};

} // namespace sensor
} // namespace core
} // namespace shift

#endif
