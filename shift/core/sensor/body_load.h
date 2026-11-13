#ifndef SHIFT_SENSOR_BODY_LOAD_H_
#define SHIFT_SENSOR_BODY_LOAD_H_

#include <memory>
#include <string>
#include <vector>

#include "shift/core/sensor/base.h"

namespace shift {
namespace core {
namespace body {
class Base;
}
namespace sensor {

/*!
@brief reports the loads the force modules put on bodies, such as the load
the links of a track put on each roller

At each output time it reports, for each body named in bodies (any body in the model, found by
name as Context::FindActiveBody does: its own assembly first, then a fully
qualified name, then the one body of that name), the force and
the moment that the force modules applied to it (contact, soil, springs and
the rest; not gravity or the joints' forces) in the last evaluation of the
forces, in the fixed frame: \<body\>_fx, _fy, _fz (N) and \<body\>_mx, _my,
_mz (N m, about the body's origin). The last evaluation is that of the
integrator's last step, which may be a little past the output time. Bodies
joined rigidly move as one group, and the load is that of the group
(Context::GetBodyLoadIndex).

The loads are complete only once every force module has run, after the
sensors are updated for the controllers, so the sensor has no signal for
controllers (get_value is 0); it reports to the loggers.

JSON:
\code
{"type": "body_load", "name": "roller_loads",
 "bodies": ["roller1", "roller2", "roller3"]}
\endcode
*/
class BodyLoad : public Base {
public:
  BodyLoad()
    : Base() {
    DeclareParameters();
  }
  BodyLoad(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~BodyLoad() override {}

  static std::unique_ptr<BodyLoad> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Update(const double*) override {}

  void UpdateOutputs(double t, const double* y) override;

protected:
  void DeclareParameters() override;

private:
  /*! \brief the names of the outputs, in the order of their values */
  std::vector<std::string> OutputNames() const;
  std::vector<std::string> body_names_;
  std::vector<body::Base*> bodies_;
  /* fx, fy, fz, mx, my, mz of each body, of the last UpdateOutputs */
  std::vector<double> loads_;

  static bool registered_;
};

} // namespace sensor
} // namespace core
} // namespace shift

#endif
