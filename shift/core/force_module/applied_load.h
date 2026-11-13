#ifndef SHIFT_FORCEMODULE_APPLIED_LOAD_H_
#define SHIFT_FORCEMODULE_APPLIED_LOAD_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "ksl/vec3.h"

#include "shift/core/force_module/base.h"
#include "shift/core/force_module/table.h"

namespace shift {
namespace core {

class Frame;

namespace body {
class Base;
}

namespace force_module {

/*!
@brief A force and moment applied to a body, or between two bodies

The load \f$s(t) (\underline{f}, \underline{m})\f$, from force and moment, acts at
the origin of a frame on body, as for a weight, a wind load or a test load.
Its coordinates are those of the frame ("frame", the default: it turns with the
body) or of the fixed frame ("fixed"). With reaction_body the opposite load
acts on that body at the same point, as an actuator between two bodies would.

The scale \f$s(t)\f$ is the product of scale, which a schedule can change, of
scale_curve or scale_file over time if given, and of a pulse if pulse_duration is
given. Between its points a curve is a smooth curve that does not overshoot
them, and before and after them it holds the end values. The pulse is an
impulse in time, smooth so that the integrator sees no jump: from pulse_start
over the time \f$t_p\f$ it is \f$1 - \cos(2 \pi (t - t_0) / t_p)\f$, zero outside.
Its integral over time is \f$t_p\f$, so that force = impulse / \f$t_p\f$ gives that
impulse. A variable step integrator knows nothing of a pulse ahead of it and
can step right over it: keep the integrator's hmax below pulse_duration.

Outputs: load_scale, the scale s(t).

JSON (SI units):
\code
{"type": "applied_load", "name": "wind", "body": "cab", "frame": "roof",
 "coordinates": "fixed", "force": [0, 800, 0]}
{"type": "applied_load", "name": "hammer", "body": "arm", "force": [0, 0, 5000],
 "pulse_start": 1.0, "pulse_duration": 0.01}
\endcode
*/
class AppliedLoad : public Base {
public:
  AppliedLoad()
    : Base() {
    DeclareParameters();
  }
  AppliedLoad(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~AppliedLoad() override {}

  static std::unique_ptr<AppliedLoad> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  void UpdateOutputs(double t, const double* y) override;

  /*! \brief the scale s at the time t */
  double Scale(double t) const;

protected:
  void DeclareParameters() override;

private:
  static bool registered_;

  std::string body_name_, frame_name_ = "origin", reaction_body_name_;
  std::string coordinates_ = "frame";
  ksl_vec3_t force_ = {{0.0, 0.0, 0.0}};
  ksl_vec3_t moment_ = {{0.0, 0.0, 0.0}};
  double scale_ = 1.0;
  std::vector<double> scale_curve_values_;
  std::string scale_file_;
  double pulse_start_ = 0.0, pulse_duration_ = 0.0;

  body::Base* body_ = nullptr;
  body::Base* reaction_body_ = nullptr;
  const Frame* frame_ = nullptr;
  int load_index_ = -2, reaction_index_ = -2;
  Table curve_;
  double output_scale_ = 0.0;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
