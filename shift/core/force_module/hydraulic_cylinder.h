#ifndef SHIFT_FORCEMODULE_HYDRAULIC_CYLINDER_H_
#define SHIFT_FORCEMODULE_HYDRAULIC_CYLINDER_H_

#include <memory>
#include <string>
#include <vector>

#include "shift/core/force_module/base.h"

namespace shift {
namespace core {

namespace joint {
class Base;
}

namespace force_module {

/*!
@brief A double-acting hydraulic cylinder on a prismatic joint, fed by a
four-way proportional valve

The pressures in the head side (the full bore) and the rod side (the
annulus) are two state variables. Each changes with the flow into its side
less the rate its volume grows, over the side's compliance (Merritt,
Hydraulic Control Systems, 1967):

\f[
\begin{aligned}
\dot{p}_h &= \frac{\beta}{v_h} (q_h - a_h \dot{x} - q_l + q_{rh}) \\
\dot{p}_r &= \frac{\beta}{v_r} (q_r + a_r \dot{x} + q_l + q_{rr})
\end{aligned}
\f]

where \f$x\f$ is the extension of the cylinder from fully retracted,
\f$a_h\f$ and \f$a_r\f$ the head and annulus areas,
\f$v_h = v_{h0} + a_h x\f$ and \f$v_r = v_{r0} + a_r (\mathrm{stroke} - x)\f$ the
volumes of oil on each side, \f$\beta\f$ the bulk modulus of the oil, and \f$q_l\f$
the leakage past the piston from head to rod side.

The valve opening \f$u\f$, from -1 to 1, connects the supply to the head side and
the rod side to the tank for u > 0 (extending), and the other way for u < 0.
The flow through each open edge is that of an orifice, in proportion to the
opening and to the square root of the pressure drop,

\f[
q = |u| \, q_{rated} \sqrt{\Delta p / \Delta p_{rated}},
\f]

with the square root made smooth near zero drop, where the flow becomes
laminar. A relief valve on each side lets oil out to the tank above the
relief pressure, and a makeup (anti-cavitation) check valve lets oil in
from the tank below the tank pressure; both open smoothly over a small band
of pressure (\f$q_{rh}, q_{rr}\f$). All flows are continuous functions of the state,
so the variable step integrators see no jumps.

The force of the cylinder, along the extension,

\f[
f = p_h a_h - p_r a_r - c \dot{x} - f_{stop},
\f]

is applied to the joint. \f$f_{stop}\f$ holds the piston at the ends of its
stroke: \f$k d (1 + e \dot{d})\f$ for a penetration \f$d\f$ past an end, never
pulling (Hunt and Crossley's contact force).

JSON:
\code
{"type": "hydraulic_cylinder", "name": "boom_cylinder",
 "joint": "M_boom_cylinder_to_boom_rod",
 "retracted_position": 2.0, "stroke": 1.4,
 "bore_diameter": 0.14, "rod_diameter": 0.1,
 "supply_pressure": 30e6, "relief_pressure": 33e6,
 "rated_flow": 3e-3,
 "command": 0,
 "end_stop_stiffness": 1e8, "end_stop_damping": 5}
\endcode

- joint: name of the prismatic joint, in the same assembly, between the
  barrel and the rod.
- direction (optional, default 1): 1 if the joint's coordinate grows as the
  cylinder extends, -1 if it shrinks.
- retracted_position: the joint's coordinate with the cylinder fully
  retracted; stroke: the travel from fully retracted to fully extended.
- bore_diameter, rod_diameter.
- dead_volumes (optional): [V_h0, V_r0], the oil in each side at the end of
  its stroke and in its lines; default 5 % of the swept volume each.
- bulk_modulus (optional, default 1.5e9 Pa): of the oil, with any air and
  the stretch of the hoses.
- supply_pressure, relief_pressure, tank_pressure (optional, default 0).
- rated_flow: the flow through one fully open edge of the valve at the
  rated_pressure_drop (optional, default 5e5 Pa) across it.
- transition_pressure (optional, default 5 % of rated_pressure_drop): the
  pressure drop below which the flow through the valve becomes laminar.
- command (optional, default 0, closed): the valve opening, -1 to 1. It
  can be changed as the model runs: by set_command, or by a schedule
  controller (see controller::Schedule) that follows control points in time,
  such as "force_modules/boom_cylinder/command". supply_pressure, leakage,
  friction, end_stop_stiffness and end_stop_damping can also be changed as the
  model runs.
- relief_flow_gain (optional, default rated_flow / (0.1 relief_pressure)):
  the flow out of the relief valves per pascal over the relief pressure;
  makeup_flow_gain (optional, default rated_flow / 1e6 Pa): the flow in
  through the makeup valves per pascal under the tank pressure.
- leakage (optional, default 0): the flow past the piston per pascal of
  difference.
- friction (optional, default 0): viscous friction of the seals, N s/m.
- end_stop_stiffness (optional, default 0: no stops), end_stop_damping
  (optional, default 0, s/m).
- initial_pressures (optional, default the tank pressure): [p_h, p_r].
- hold (optional, default false): start with the pressures that hold the
  cylinder still, in place of initial_pressures. Before the simulation
  starts, the forces of all holding cylinders that give their joints zero
  acceleration are found together from the equations of motion (see
  Context::HoldInitialStates); each cylinder's force is then made by the
  pressure on one side, the other at the tank pressure. Where cylinders
  share a load, as a pair side by side, they share it equally.

Pressures, flows and lengths are in the model's units (pascals, cubic
metres per second, metres, for SI).
*/
class HydraulicCylinder : public Base {
public:
  HydraulicCylinder()
    : Base() {
    Initialize();
  }
  HydraulicCylinder(std::string name)
    : Base(name) {
    Initialize();
  }
  virtual ~HydraulicCylinder() override {}

  static std::unique_ptr<HydraulicCylinder> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;

  bool holds() const override {
    return hold_;
  }
  const joint::Base* get_held_joint() const override {
    return joint_;
  }
  void set_hold_force(bool active, double force) override {
    hold_force_active_ = active;
    hold_force_ = force;
  }
  void Hold(double force, const double* y) override;
  void Service(const double* y, double* dy) override;

  /*! \brief logged outputs: head_pressure, rod_pressure, force,
   * extension, opening, head_flow and rod_flow */
  std::vector<std::string> get_output_names() const override;
  void UpdateOutputs(double t, const double* y) override;
  void AppendOutputs(std::vector<double>& values) const override;

  /*! \brief the valve opening, -1 to 1 (the parameter command) */
  void set_command(double u) {
    command_ = u;
  }

  /*! \brief as of the last Service or UpdateOutputs: */
  double get_head_pressure() const {
    return p_head_;
  }
  double get_rod_pressure() const {
    return p_rod_;
  }
  /*! \brief the force of the cylinder along its extension */
  double get_force() const {
    return force_;
  }
  double get_extension() const {
    return extension_;
  }
  /*! \brief the valve opening, -1 to 1 */
  double get_opening() const {
    return opening_;
  }
  /*! \brief flows through the valve into the head and rod sides */
  double get_head_flow() const {
    return q_head_;
  }
  double get_rod_flow() const {
    return q_rod_;
  }
  double get_head_area() const {
    return area_head_;
  }
  double get_rod_area() const {
    return area_rod_;
  }

private:
  void Initialize() {
    equation_count_ = 2;
    q_0_ = {0.0, 0.0};
    DeclareParameters();
  }
  virtual void DeclareParameters() override;
  /*! \brief the pressures, flows and force at time t for the state y, and
   * the pressure rates in dy unless it is null */
  void Evaluate(const double* y, double* dy);
  /*! \brief the flow through an open valve edge for the pressure drop dp,
   * per unit opening */
  double EdgeFlow(double dp) const;
  /*! \brief smooth max(0, p) opening over the band w */
  static double Ramp(double p, double w);

  std::string joint_name_;
  int direction_ = 1;
  double retracted_position_ = 0.0;
  double stroke_ = 0.0;
  double bore_diameter_ = 0.0;
  double rod_diameter_ = 0.0;
  std::vector<double> dead_volumes_;
  double bulk_modulus_ = 1.5e9;
  double supply_pressure_ = 0.0;
  double relief_pressure_ = 0.0;
  double tank_pressure_ = 0.0;
  double rated_flow_ = 0.0;
  double rated_pressure_drop_ = 5e5;
  double transition_pressure_ = -1.0;
  double relief_flow_gain_ = -1.0;
  double makeup_flow_gain_ = -1.0;
  double leakage_ = 0.0;
  double friction_ = 0.0;
  double end_stop_stiffness_ = 0.0;
  double end_stop_damping_ = 0.0;
  std::vector<double> initial_pressures_;
  bool hold_ = false;
  bool hold_force_active_ = false;
  double hold_force_ = 0.0; //!< generalized force on the joint, while active

  joint::Base* joint_ = nullptr;
  double command_ = 0.0;
  double area_head_ = 0.0;
  double area_rod_ = 0.0;
  double volume_head_0_ = 0.0;
  double volume_rod_0_ = 0.0;

  double p_head_ = 0.0, p_rod_ = 0.0, force_ = 0.0, extension_ = 0.0;
  double opening_ = 0.0, q_head_ = 0.0, q_rod_ = 0.0;

  static bool registered_;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
