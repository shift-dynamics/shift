#ifndef SHIFT_SENSOR_MARKER_H_
#define SHIFT_SENSOR_MARKER_H_

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
@brief reports the pose and velocity of coordinate frames on bodies, as
markers on a test rig would

At each output time it reports, for each frame of frames (pairs of a body and
one of its frames; the body is any in the model, found by name as
Context::FindActiveBody does), in the fixed frame:

- \<body\>_\<frame\>_x, _y, _z: the position of the frame's origin (m)
- _qw, _qx, _qy, _qz: the orientation of the frame, the unit quaternion of
  the rotation from the fixed axes to the frame's
- _roll, _pitch, _yaw: the same orientation as the angles (rad) of the
  rotations about z (yaw), then the new y (pitch), then the new x (roll)
  that turn the fixed axes into the frame's, \f$R = R_z(\psi) R_y(\theta)
  R_x(\phi)\f$
- _vx, _vy, _vz: the velocity of the frame's origin (m/s)
- _wx, _wy, _wz: the angular velocity of the frame (rad/s)

With coordinate_system "local" the velocities are given in the axes of the
frame instead. The sensor reports to the loggers and has no signal for
controllers (get_value is 0); the frame sensor measures one component for a
controller.

JSON:
\code
{"type": "marker", "name": "shoes",
 "frames": [["link10", "origin"], ["link11", "origin"]],
 "coordinate_system": "global"}
\endcode
*/
class Marker : public Base {
public:
  Marker()
    : Base() {
    DeclareParameters();
  }
  Marker(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Marker() override {}

  static std::unique_ptr<Marker> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Update(const double*) override {}

  void UpdateOutputs(double t, const double* y) override;

  /*! \brief the number of values reported for each frame */
  static constexpr size_t kValuesPerFrame = 16;

protected:
  void DeclareParameters() override;

private:
  /*! \brief the names of the outputs, in the order of their values */
  std::vector<std::string> OutputNames() const;
  std::vector<std::vector<std::string>> frame_names_;
  std::string coordinate_system_ = "global";
  std::vector<body::Base*> bodies_;
  std::vector<shift::core::Frame*> frames_;
  bool local_ = false;
  /* kValuesPerFrame values of each frame, of the last UpdateOutputs */
  std::vector<double> values_;

  static bool registered_;
};

} // namespace sensor
} // namespace core
} // namespace shift

#endif
