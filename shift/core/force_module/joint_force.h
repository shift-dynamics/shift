#ifndef SHIFT_FORCEMODULE_JOINT_FORCE_H_
#define SHIFT_FORCEMODULE_JOINT_FORCE_H_

#include <memory>
#include <string>
#include <vector>

#include "shift/core/force_module/base.h"
#include "shift/core/force_module/table.h"

namespace shift {
namespace core {

namespace joint {
class Base;
}

namespace force_module {

/*!
@brief Coulomb and viscous friction on a joint of one coordinate, which can
hold it at rest

With the joint's rate \f$v\f$ (the force and rate are a torque and an angular
velocity on a revolute joint), and without bristle_stiffness, the friction
force on the joint is

\f[
f = -\left( f_c \frac{v}{\sqrt{v^2 + v_s^2}} + c_v v \right)
\f]

with the friction \f$f_c\f$, the slip_speed \f$v_s\f$ and the viscous_friction
\f$c_v\f$. It is smooth, so the joint creeps at rest under a steady load.

With bristle_stiffness \f$\sigma_0\f$, the friction holds at rest: the LuGre model
(Canudas de Wit, Olsson, Astrom and Lischinsky, 1995). The bristles have the
deflection \f$z\f$, a state,

\f[
\dot{z} = v - \frac{\sigma_0 |v|}{g(v)} z, \quad
g(v) = f_c + (f_s - f_c) e^{-(v / v_b)^2}, \quad
f = -\left( \sigma_0 z + \sigma_1 \dot{z} + c_v v \right)
\f]

with the static_friction \f$f_s\f$, the stribeck_velocity \f$v_b\f$ and the
bristle_damping \f$\sigma_1\f$.

Outputs: friction_force (the force on the joint), and with bristles stick,
the share of the static friction in use, \f$\sigma_0 |z| / f_s\f$.

JSON:
\code
{"type": "joint_friction", "name": "hinge_friction", "joint": "hinge",
 "friction": 2.0, "static_friction": 3.0, "viscous_friction": 0.1,
 "bristle_stiffness": 5000}
\endcode
*/
class JointFriction : public Base {
public:
  JointFriction()
    : Base() {
    DeclareParameters();
  }
  JointFriction(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~JointFriction() override {}

  static std::unique_ptr<JointFriction> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  void UpdateOutputs(double t, const double* y) override;

  double get_friction() const {
    return force_;
  }
  double get_stick() const {
    return stick_;
  }

protected:
  void DeclareParameters() override;

private:
  void Evaluate(const double* y, double* dy);

  static bool registered_;

  std::string joint_name_;
  double friction_ = 0.0;
  double static_friction_ = -1.0; //!< negative: the friction
  double viscous_friction_ = 0.0;
  double slip_speed_ = 0.01;
  double bristle_stiffness_ = 0.0;
  double bristle_damping_ = 0.5;
  double stribeck_velocity_ = 0.01;

  joint::Base* joint_ = nullptr;
  double force_ = 0.0, stick_ = 0.0;
};

/*!
@brief A force on a joint of one coordinate from a measured map of two
variables

Each of the two variables is the coordinate or the rate of a joint: x of the
joint itself and y of it or of y_joint. The map gives \f$f(x, y)\f$ at the
points of a grid, and between them it is smooth, a monotone cubic
(force_module::Table) along x in each row and then along y through those
values, continuing with the end slopes outside the grid. The force acts on
joint, positive along its coordinate.

The map is given by x_values, y_values and forces, a row of the forces along x
for each y, in the model or in map_file, a text file of numbers separated by
blanks, commas or line breaks: the number of x values, the number of y values,
the x values, the y values, and the forces.

JSON:
\code
{"type": "joint_map", "name": "damper", "joint": "strut", "x": "coordinate",
 "y": "rate", "x_values": [-0.1, 0, 0.1], "y_values": [-1, 0, 1],
 "forces": [800, 900, 1000, 0, 0, 0, -1000, -1100, -1200]}
\endcode

Outputs: force.
*/
class JointMap : public Base {
public:
  JointMap()
    : Base() {
    DeclareParameters();
  }
  JointMap(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~JointMap() override {}

  static std::unique_ptr<JointMap> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  void UpdateOutputs(double t, const double* y) override;

  /*! \brief the force of the map at (x, y) */
  double Force(double x, double y) const;
  double get_force() const {
    return force_;
  }

protected:
  void DeclareParameters() override;

private:
  double Variable(const std::string& kind, const joint::Base* joint,
                  const double* y) const;

  static bool registered_;

  std::string joint_name_, y_joint_name_;
  std::string x_kind_ = "coordinate", y_kind_ = "rate";
  std::vector<double> x_values_, y_values_, forces_;
  std::string map_file_;

  joint::Base* joint_ = nullptr;
  joint::Base* y_joint_ = nullptr;
  std::vector<Table> rows_; //!< along x, one for each y
  double force_ = 0.0;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
