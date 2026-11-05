#ifndef SHIFT_CONTROLLER_SCHEDULE_H_
#define SHIFT_CONTROLLER_SCHEDULE_H_

#include <memory>
#include <string>
#include <vector>

#include "shift/core/controller/base.h"
#include "shift/core/force_module/table.h"

namespace shift {
namespace core {
namespace controller {

/*!
@brief changes parameters of other components as a function of time

A schedule gives a value at each of its control points, [t0, v0, t1, v1,
...], and sets the parameters it targets to the value at the time of each
evaluation of the equations of motion, before the other controllers and the
force modules that read them, wherever it is in the model file. Before the first point and after the last it holds
that point's value. The value between points follows interpolation:

- "linear": straight lines between the points. The value is continuous, its
  rate jumps at each point.
- "cubic": a cubic in each interval whose slopes at the points are chosen so
  that the curve does not overshoot the points, rising where they rise and
  level where they are level (the monotone cubic of Fritsch and Carlson,
  force_module::Table). The value and its rate are continuous.
- "step": the value of the last point passed, so that it jumps at each point.
  A jump in a parameter, such as a force or a command, is a jump in the
  equations that the variable step integrator has to find: it takes small
  steps around it. With transition_time > 0 each jump is made over that
  time, as a smooth cubic from the old value to the new, which is smooth for
  the integrator; transition_time must be less than the time between the
  points.

Targets are the parameters of other components, given by their path from the
assembly that holds the schedule, the keys of the model file: for a force
module "force_modules/boom_cylinder/command", for a controller
"controllers/boom_speed/target", for the collision geometry of a body
"bodies/link1/collision_geometry/link1/..."; into another assembly, by
"assemblies/<name>/" in front. One schedule can set several targets to the
same value, as for two cylinders on one valve. An entry of a vector or a 3
vector parameter is selected by its index at the end of the path, as in
"force_modules/mount/stiffness/2". The targets are numbers (a double, an
entry of a vector of doubles or of a 3 vector).

A parameter can be a target only if its component has declared that it reads
it each time it evaluates its forces and computes nothing from it only when it
loads (parameter::Value::set_runtime); the schedule refuses others when the
model is loaded, with a message saying so.

JSON:
\code
{"type": "schedule", "name": "boom_valve", "interpolation": "cubic",
 "points": [0, 0, 0.2, 0, 0.5, -0.4, 1.8, -0.4, 2.1, 0],
 "targets": ["force_modules/boom_cylinder_left/command",
             "force_modules/boom_cylinder_right/command"]}
\endcode
*/
class Schedule : public Base {
public:
  enum class Interpolation { kLinear, kCubic, kStep };

  Schedule()
    : Base() {
    type_ = "schedule";
    DeclareParameters();
  }

  Schedule(std::string name)
    : Base(name) {
    type_ = "schedule";
    DeclareParameters();
  }

  virtual ~Schedule() override {}

  static std::unique_ptr<Schedule> Create();

  static bool is_registered() {
    return registered_;
  }

  void DeclareParameters() override;
  void Connect() override;
  void Service(const double* y, double* dy) override;

  /*! \brief schedules are evaluated before the other controllers, so that
   * what they set is what the other controllers and the force modules read in
   * the same evaluation, wherever the schedule is in the model file */
  bool runs_first() const override {
    return true;
  }

  /*! \brief the value of the schedule at time t */
  double Evaluate(double t) const;

  /*! \brief the value set at the last evaluation */
  double get_value() const {
    return value_;
  }

private:
  /*! a target number: a double, or an entry of a vector of doubles or of a
   * 3 vector */
  struct Target {
    std::string path;
    double* scalar = nullptr;
    std::vector<double>* vector = nullptr;
    size_t index = 0;

    double* Pointer() const {
      return vector ? vector->data() + index : scalar;
    }
  };

  /*! \brief resolve a path to the number it names; throws if it names none,
   * or one that is not changeable at run time */
  Target Resolve(const std::string& path) const;

  static bool registered_;

  std::string interpolation_name_ = "linear";
  std::vector<double> points_;
  double transition_time_ = 0.0;
  std::vector<std::string> target_paths_;

  Interpolation interpolation_ = Interpolation::kLinear;
  std::vector<double> t_, v_;
  force_module::Table table_;
  std::vector<Target> targets_;
  double value_ = 0.0;
};

} // namespace controller
} // namespace core
} // namespace shift

#endif
