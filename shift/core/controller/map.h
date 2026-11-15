#ifndef SHIFT_CONTROLLER_MAP_H_
#define SHIFT_CONTROLLER_MAP_H_

#include <memory>
#include <string>

#include "shift/core/controller/schedule.h"

namespace shift {
namespace core {
namespace controller {

/*!
@brief sets parameters of other components from an output of a component,
through a curve

A map reads an output of a component (parameter::List::DeclareOutput), such
as the value of a schedule, the signal of a sensor or the pressure of a
cylinder, of its assembly or, by the fully qualified name
(<assembly>__<name>), of another (Assembly::FindComponent), and sets the
parameters it targets to the value of a curve at it:
the points \f$[x_0, v_0, x_1, v_1, \ldots]\f$ give the value \f$v\f$ at each
value \f$x\f$ of the output, between them as interpolation gives (see
Schedule: "linear", "cubic" or "step"), and beyond them the value of the
nearest point. Without points the value is the output itself.

Its targets are given and checked as a schedule's, by their paths from the
assembly of the map, and must be changeable while the model runs; an input
of another component, such as the intensity of a light
("bodies/chassis/lights/brake_lamp/intensity"), is one. The map is evaluated
with the other controllers, in the order of the model file, after the
schedules; it reads an output of another controller of that kind as that
controller left it, so it should come after it in the model file.

JSON:
\code
{"type": "map", "name": "brake_lights", "component": "brake_pedal",
 "output": "value", "points": [0.0, 0.0, 0.05, 1.0],
 "targets": ["bodies/chassis/lights/left_brake_lamp/intensity",
             "bodies/chassis/lights/right_brake_lamp/intensity"]}
\endcode
*/
class Map : public Schedule {
public:
  Map()
    : Schedule() {
    Initialize();
  }

  Map(std::string name)
    : Schedule(name) {
    Initialize();
  }

  virtual ~Map() override {}

  static std::unique_ptr<Map> Create();

  static bool is_registered() {
    return registered_;
  }

  void DeclareParameters() override;
  void Connect() override;
  void Service(const double* y, double* dy) override;
  void UpdateOutputs(double t, const double* y) override;

  /*! \brief evaluated with the other controllers, not before them */
  bool runs_first() const override {
    return false;
  }

  /*! \brief the value of the curve at x, the output it reads */
  double Curve(double x) const;

  /*! \brief the output read at the last evaluation */
  double get_input() const {
    return input_;
  }

private:
  void Initialize() {
    type_ = "map";
    label_ = "Map";
    DeclareParameters();
  }
  /*! \brief reads the output and sets the targets */
  void Apply();

  static bool registered_;

  std::string component_name_;
  std::string output_name_;
  const double* output_ = nullptr;
  double input_ = 0.0;
  bool identity_ = false; //!< no points: the value is the output
};

} // namespace controller
} // namespace core
} // namespace shift

#endif
