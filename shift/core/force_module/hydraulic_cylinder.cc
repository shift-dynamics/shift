#include "shift/core/force_module/hydraulic_cylinder.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "shift/core/assembly.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/joint/prismatic.h"
#include "shift/core/log.h"

bool shift::core::force_module::HydraulicCylinder::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "hydraulic_cylinder", shift::core::force_module::HydraulicCylinder::Create);

std::unique_ptr<shift::core::force_module::HydraulicCylinder>
shift::core::force_module::HydraulicCylinder::Create() {
  return std::make_unique<shift::core::force_module::HydraulicCylinder>();
}

void shift::core::force_module::HydraulicCylinder::DeclareParameters() {
  DeclareParameter("joint", &joint_name_,
                   "name of the prismatic joint between barrel and rod, in "
                   "the same assembly",
                   true);
  DeclareParameter("direction", &direction_,
                   "1 if the joint coordinate grows as the cylinder extends, "
                   "-1 if it shrinks; optional, default 1",
                   false);
  DeclareParameter("retracted_position", &retracted_position_,
                   "joint coordinate with the cylinder fully retracted", true);
  DeclareParameter("stroke", &stroke_,
                   "travel from fully retracted to fully extended", true);
  DeclareParameter("bore_diameter", &bore_diameter_, "bore diameter", true);
  DeclareParameter("rod_diameter", &rod_diameter_, "rod diameter", true);
  DeclareParameter("dead_volumes", &dead_volumes_,
                   "[head, rod] oil volumes at the ends of the stroke and in "
                   "the lines; optional, default 5 % of the swept volume",
                   false)
    .set_row_dim(2);
  DeclareParameter("bulk_modulus", &bulk_modulus_,
                   "effective bulk modulus of the oil; optional, default "
                   "1.5e9 Pa",
                   false);
  DeclareParameter("supply_pressure", &supply_pressure_,
                   "pressure at the valve's supply port", true)
    .set_runtime();
  DeclareParameter("relief_pressure", &relief_pressure_,
                   "opening pressure of the relief valves", true);
  DeclareParameter("tank_pressure", &tank_pressure_,
                   "pressure at the valve's tank port; optional, default 0",
                   false);
  DeclareParameter("rated_flow", &rated_flow_,
                   "flow through one fully open valve edge at the rated "
                   "pressure drop",
                   true);
  DeclareParameter("rated_pressure_drop", &rated_pressure_drop_,
                   "pressure drop across an edge at the rated flow; optional, "
                   "default 5e5 Pa",
                   false);
  DeclareParameter("transition_pressure", &transition_pressure_,
                   "pressure drop below which the valve flow is laminar; "
                   "optional, default 5 % of rated_pressure_drop",
                   false);
  DeclareInput("command", &command_,
               "valve opening, -1 to 1 (beyond, it is clipped); optional, "
               "default 0 (closed); set as the model runs by a PID or "
               "schedule controller, or set_command");
  DeclareParameter("relief_flow_gain", &relief_flow_gain_,
                   "relief flow per unit pressure over the relief pressure; "
                   "optional, default rated_flow / (0.1 relief_pressure)",
                   false);
  DeclareParameter("makeup_flow_gain", &makeup_flow_gain_,
                   "makeup flow per unit pressure under the tank pressure; "
                   "optional, default rated_flow / 1e6 Pa",
                   false);
  DeclareParameter("leakage", &leakage_,
                   "flow past the piston per unit pressure difference; "
                   "optional, default 0",
                   false)
    .set_runtime();
  DeclareParameter("friction", &friction_,
                   "viscous friction of the seals; optional, default 0",
                   false)
    .set_runtime();
  DeclareParameter("end_stop_stiffness", &end_stop_stiffness_,
                   "stiffness of the stops at the ends of the stroke; "
                   "optional, default 0 (no stops)",
                   false)
    .set_runtime();
  DeclareParameter("end_stop_damping", &end_stop_damping_,
                   "Hunt-Crossley damping of the stops, per unit "
                   "penetration rate; optional, default 0",
                   false)
    .set_runtime();
  DeclareParameter("hold", &hold_,
                   "start with the pressures that hold the cylinder still, "
                   "in place of initial_pressures; optional, default false",
                   false);
  DeclareParameter("initial_pressures", &initial_pressures_,
                   "[head, rod] pressures at the start; optional, default "
                   "the tank pressure",
                   false)
    .set_row_dim(2);
  DeclareOutput("head_pressure", &p_head_, "pressure in the head chamber");
  DeclareOutput("rod_pressure", &p_rod_, "pressure in the rod chamber");
  DeclareOutput("force", &force_, "force of the cylinder along its extension");
  DeclareOutput("extension", &extension_, "extension of the cylinder");
  DeclareOutput("opening", &opening_, "opening of the valve, -1 to 1");
  DeclareOutput("head_flow", &q_head_, "flow into the head chamber");
  DeclareOutput("rod_flow", &q_rod_, "flow into the rod chamber");
}

void shift::core::force_module::HydraulicCylinder::Connect() {
  if(parent_assembly_ptr_ == nullptr) {
    throw shift::core::Exception("No parent assembly set for hydraulic "
                                 "cylinder " + get_name());
  }
  try {
    joint_ = (*parent_assembly_ptr_)["joints"][joint_name_]
               .get_data<shift::core::joint::Base>();
  } catch(...) {
    joint_ = nullptr;
  }
  if(dynamic_cast<shift::core::joint::Prismatic*>(joint_) == nullptr) {
    throw shift::core::Exception("Hydraulic cylinder " + get_name() +
                                 ": joint '" + joint_name_ +
                                 "' not found in its assembly or not a "
                                 "prismatic joint");
  }
}

void shift::core::force_module::HydraulicCylinder::Load() {
  const std::string prefix = "Hydraulic cylinder " + get_name() + ": ";
  if(direction_ != 1 && direction_ != -1) {
    throw shift::core::Exception(prefix + "direction must be 1 or -1");
  }
  if(!(stroke_ > 0.0) || !(bore_diameter_ > 0.0) || !(rod_diameter_ >= 0.0) ||
     !(rod_diameter_ < bore_diameter_)) {
    throw shift::core::Exception(prefix + "needs stroke > 0 and bore_diameter "
                                 "> rod_diameter >= 0");
  }
  if(!(bulk_modulus_ > 0.0) || !(rated_flow_ > 0.0) ||
     !(rated_pressure_drop_ > 0.0)) {
    throw shift::core::Exception(prefix + "bulk_modulus, rated_flow and "
                                 "rated_pressure_drop must be positive");
  }
  if(!(relief_pressure_ > tank_pressure_)) {
    throw shift::core::Exception(prefix + "relief_pressure must be above "
                                 "tank_pressure");
  }
  area_head_ = M_PI * bore_diameter_ * bore_diameter_ / 4.0;
  area_rod_ = area_head_ - M_PI * rod_diameter_ * rod_diameter_ / 4.0;
  if(dead_volumes_.empty()) {
    volume_head_0_ = 0.05 * area_head_ * stroke_;
    volume_rod_0_ = 0.05 * area_rod_ * stroke_;
  } else {
    volume_head_0_ = dead_volumes_[0];
    volume_rod_0_ = dead_volumes_[1];
  }
  if(!(volume_head_0_ > 0.0) || !(volume_rod_0_ > 0.0)) {
    throw shift::core::Exception(prefix + "dead_volumes must be positive");
  }
  if(transition_pressure_ < 0.0) {
    transition_pressure_ = 0.05 * rated_pressure_drop_;
  }
  if(relief_flow_gain_ < 0.0) {
    relief_flow_gain_ = rated_flow_ / (0.1 * relief_pressure_);
  }
  if(makeup_flow_gain_ < 0.0) {
    makeup_flow_gain_ = rated_flow_ / 1e6;
  }
  if(hold_ && !initial_pressures_.empty()) {
    throw shift::core::Exception(prefix + "give hold or initial_pressures, "
                                 "not both");
  }
  if(initial_pressures_.empty()) {
    q_0_ = {tank_pressure_, tank_pressure_};
  } else {
    q_0_ = {initial_pressures_[0], initial_pressures_[1]};
  }
  p_head_ = q_0_[0];
  p_rod_ = q_0_[1];
}

double shift::core::force_module::HydraulicCylinder::Ramp(double p,
                                                          double w) {
  if(p <= 0.0) {
    return 0.0;
  }
  if(p < w) {
    return 0.5 * p * p / w;
  }
  return p - 0.5 * w;
}

double shift::core::force_module::HydraulicCylinder::EdgeFlow(
  double dp) const {
  /* sqrt(|dp|) sign(dp), made smooth through zero:
  dp / (dp^2 + dp_t^2)^(1/4) */
  const double t = transition_pressure_;
  return rated_flow_ * dp /
         (std::sqrt(rated_pressure_drop_) * std::pow(dp * dp + t * t, 0.25));
}

void shift::core::force_module::HydraulicCylinder::Service(const double* y,
                                                           double* dy) {
  if(hold_force_active_) {
    dy[state_offset_] = 0.0;
    dy[state_offset_ + 1] = 0.0;
    context_->get_joint_working_forces()[joint_->get_dq_index()] +=
      hold_force_;
    return;
  }
  Evaluate(y, dy);
  context_->get_joint_working_forces()[joint_->get_dq_index()] +=
    direction_ * force_;
}

void shift::core::force_module::HydraulicCylinder::UpdateOutputs(
  double, const double* y) {
  Evaluate(y, nullptr);
}

void shift::core::force_module::HydraulicCylinder::Evaluate(const double* y,
                                                            double* dy) {
  assert(joint_ && context_);
  const double q = y[joint_->get_q_index()];
  const double dq = y[context_->get_q_count() + joint_->get_dq_index()];
  const double x = direction_ * (q - retracted_position_);
  const double dx = direction_ * dq;
  p_head_ = y[state_offset_];
  p_rod_ = y[state_offset_ + 1];
  extension_ = x;

  /* the valve: u > 0 connects supply to head and rod to tank */
  const double u = std::clamp(command_, -1.0, 1.0);
  opening_ = u;
  if(u >= 0.0) {
    q_head_ = u * EdgeFlow(supply_pressure_ - p_head_);
    q_rod_ = -u * EdgeFlow(p_rod_ - tank_pressure_);
  } else {
    q_head_ = u * EdgeFlow(p_head_ - tank_pressure_);
    q_rod_ = -u * EdgeFlow(supply_pressure_ - p_rod_);
  }

  /* relief valves to the tank and makeup check valves from it, each opening
  over a band of 2 % of the relief pressure */
  const double band = 0.02 * relief_pressure_;
  auto valves = [&](double p) {
    return makeup_flow_gain_ * Ramp(tank_pressure_ - p, band) -
           relief_flow_gain_ * Ramp(p - relief_pressure_, band);
  };
  const double leak = leakage_ * (p_head_ - p_rod_);

  /* the volumes, kept from vanishing past the stops */
  const double volume_head =
    std::max(volume_head_0_ + area_head_ * x, 0.1 * volume_head_0_);
  const double volume_rod =
    std::max(volume_rod_0_ + area_rod_ * (stroke_ - x), 0.1 * volume_rod_0_);
  if(dy) {
    dy[state_offset_] = bulk_modulus_ / volume_head *
                        (q_head_ - area_head_ * dx - leak + valves(p_head_));
    dy[state_offset_ + 1] = bulk_modulus_ / volume_rod *
                            (q_rod_ + area_rod_ * dx + leak + valves(p_rod_));
  }

  /* the force along the extension */
  double stop = 0.0;
  if(end_stop_stiffness_ > 0.0) {
    const double below = -x;
    const double above = x - stroke_;
    if(below > 0.0) {
      stop -= std::max(
        0.0, end_stop_stiffness_ * below * (1.0 - end_stop_damping_ * dx));
    }
    if(above > 0.0) {
      stop += std::max(
        0.0, end_stop_stiffness_ * above * (1.0 + end_stop_damping_ * dx));
    }
  }
  force_ = p_head_ * area_head_ - p_rod_ * area_rod_ - friction_ * dx - stop;
}

void shift::core::force_module::HydraulicCylinder::Hold(double force,
                                                        const double* y) {
  /* the pressures' share of the force along the extension: the joint
  receives direction * force_, and force_ is the pressures' force less the
  friction and the stops at the initial state */
  std::vector<double> state(y, y + state_offset_ + 2);
  state[state_offset_] = state[state_offset_ + 1] = 0.0;
  Evaluate(state.data(), nullptr);
  const double pressures = direction_ * force - force_;
  if(pressures >= tank_pressure_ * (area_head_ - area_rod_)) {
    q_0_ = {(pressures + tank_pressure_ * area_rod_) / area_head_,
            tank_pressure_};
  } else {
    q_0_ = {tank_pressure_,
            (tank_pressure_ * area_head_ - pressures) / area_rod_};
  }
  if(q_0_[0] > relief_pressure_ || q_0_[1] > relief_pressure_) {
    SHIFT_WARNING("Hydraulic cylinder " + get_name() + ": holding needs " +
                  std::to_string(std::max(q_0_[0], q_0_[1])) +
                  ", above the relief pressure; the load will sink");
  }
  p_head_ = q_0_[0];
  p_rod_ = q_0_[1];
}
