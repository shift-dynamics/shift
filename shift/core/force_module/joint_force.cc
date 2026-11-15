#include "shift/core/force_module/joint_force.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include "shift/core/assembly.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/joint/base.h"
#include "shift/core/joint/prismatic.h"
#include "shift/core/joint/revolute.h"

bool shift::core::force_module::JointFriction::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "joint_friction", shift::core::force_module::JointFriction::Create);

bool shift::core::force_module::JointMap::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "joint_map", shift::core::force_module::JointMap::Create);

std::unique_ptr<shift::core::force_module::JointFriction>
shift::core::force_module::JointFriction::Create() {
  return std::make_unique<shift::core::force_module::JointFriction>();
}

std::unique_ptr<shift::core::force_module::JointMap>
shift::core::force_module::JointMap::Create() {
  return std::make_unique<shift::core::force_module::JointMap>();
}

/* a revolute or prismatic joint of the assembly, by name */
static shift::core::joint::Base* FindJoint(shift::core::Assembly* assembly,
                                           const std::string& name,
                                           const std::string& prefix) {
  shift::core::joint::Base* joint = nullptr;
  try {
    joint = (*assembly)["joints"][name].get_data<shift::core::joint::Base>();
  } catch(...) {
    joint = nullptr;
  }
  if(!joint) {
    throw shift::core::Exception(prefix + "joint " + name + " not found");
  }
  if(!dynamic_cast<shift::core::joint::Revolute*>(joint) &&
     !dynamic_cast<shift::core::joint::Prismatic*>(joint)) {
    throw shift::core::Exception(prefix + "joint " + name +
                                 " must be revolute or prismatic");
  }
  return joint;
}

void shift::core::force_module::JointFriction::DeclareParameters() {
  DeclareParameter("joint", &joint_name_,
                   "name of the revolute or prismatic joint", true);
  DeclareParameter("friction", &friction_,
                   "Coulomb friction force (torque on a revolute joint)",
                   true);
  DeclareParameter("static_friction", &static_friction_,
                   "with bristles, the force the friction holds up to, at "
                   "least friction; optional, default friction",
                   false);
  DeclareParameter("viscous_friction", &viscous_friction_,
                   "force per rate; optional, default 0", false);
  DeclareParameter("slip_speed", &slip_speed_,
                   "without bristles, the rate over which the friction builds "
                   "up; optional, default 0.01",
                   false);
  DeclareParameter("bristle_stiffness", &bristle_stiffness_,
                   "force per deflection of the bristles, which makes the "
                   "friction hold at rest; optional, default 0 (the friction "
                   "creeps at rest)",
                   false);
  DeclareParameter("bristle_damping", &bristle_damping_,
                   "force per rate of the deflection of the bristles. With 0 "
                   "the bristles ring, with little to settle them; optional, "
                   "default 0.5",
                   false);
  DeclareParameter("stribeck_velocity", &stribeck_velocity_,
                   "with bristles, the rate over which the static friction "
                   "falls to friction; optional, default 0.01",
                   false);
}

void shift::core::force_module::JointFriction::Connect() {
  joint_ = FindJoint(parent_assembly_ptr_, joint_name_,
                     "Joint friction " + get_name() + ": ");
}

void shift::core::force_module::JointFriction::Load() {
  if(static_friction_ < 0.0) {
    static_friction_ = friction_;
  }
  if(friction_ < 0.0 || static_friction_ < friction_ ||
     viscous_friction_ < 0.0 || !(slip_speed_ > 0.0) ||
     bristle_stiffness_ < 0.0 || bristle_damping_ < 0.0 ||
     !(stribeck_velocity_ > 0.0)) {
    throw shift::core::Exception(
      "Joint friction " + get_name() +
      ": friction, viscous_friction, bristle_stiffness and bristle_damping "
      "must not be negative, slip_speed and stribeck_velocity must be "
      "positive, and static_friction at least friction");
  }
  equation_count_ = bristle_stiffness_ > 0.0 ? 1 : 0;
  q_0_.assign(equation_count_, 0.0);

  DeclareOutput("friction_force", &force_,
                "friction force on the joint (torque on a revolute joint)");
  if(bristle_stiffness_ > 0.0) {
    DeclareOutput("stick", &stick_,
                  "share of the static friction in use, 1 at the limit of "
                  "holding");
  }
}

void shift::core::force_module::JointFriction::Evaluate(const double* y,
                                                        double* dy) {
  const double v = y[context_->get_q_count() + joint_->get_dq_index()];
  stick_ = 0.0;
  if(bristle_stiffness_ > 0.0) {
    const double z = y[state_offset_];
    const double ratio = v / stribeck_velocity_;
    const double g =
      friction_ + (static_friction_ - friction_) * std::exp(-ratio * ratio);
    const double rate =
      g > 0.0 ? v - bristle_stiffness_ * std::fabs(v) * z / g : v;
    if(dy) {
      dy[state_offset_] = rate;
    }
    force_ = -(bristle_stiffness_ * z + bristle_damping_ * rate +
               viscous_friction_ * v);
    stick_ = static_friction_ > 0.0
               ? bristle_stiffness_ * std::fabs(z) / static_friction_
               : 0.0;
  } else {
    force_ = -(friction_ * v / std::sqrt(v * v + slip_speed_ * slip_speed_) +
               viscous_friction_ * v);
  }
}

void shift::core::force_module::JointFriction::Service(const double* y,
                                                       double* dy) {
  Evaluate(y, dy);
  context_->get_joint_working_forces()[joint_->get_dq_index()] += force_;
}

void shift::core::force_module::JointFriction::UpdateOutputs(double,
                                                             const double* y) {
  Evaluate(y, nullptr);
}

void shift::core::force_module::JointMap::DeclareParameters() {
  DeclareParameter("joint", &joint_name_,
                   "name of the revolute or prismatic joint the force acts "
                   "on, and of the joint of x",
                   true);
  DeclareParameter("y_joint", &y_joint_name_,
                   "the joint of y; optional, default joint", false);
  DeclareParameter("x", &x_kind_,
                   "\"coordinate\" (default) or \"rate\" of joint", false);
  DeclareParameter("y", &y_kind_,
                   "\"coordinate\" or \"rate\" (default) of y_joint", false);
  DeclareParameter("x_values", &x_values_, "the grid along x, increasing",
                   false)
    .set_row_dim(-1);
  DeclareParameter("y_values", &y_values_, "the grid along y, increasing",
                   false)
    .set_row_dim(-1);
  DeclareParameter("forces", &forces_,
                   "the forces, a row along x for each y", false)
    .set_row_dim(-1);
  DeclareParameter("map_file", &map_file_,
                   "text file of the number of x values, the number of y "
                   "values, x_values, y_values and forces; in place of them",
                   false);

  DeclareOutput("force", &force_,
                "force of the map on the joint (torque on a revolute joint)");
}

void shift::core::force_module::JointMap::Connect() {
  const std::string prefix = "Joint map " + get_name() + ": ";
  joint_ = FindJoint(parent_assembly_ptr_, joint_name_, prefix);
  y_joint_ =
    y_joint_name_.empty()
      ? joint_
      : FindJoint(parent_assembly_ptr_, y_joint_name_, prefix);
}

void shift::core::force_module::JointMap::Load() {
  const std::string prefix = "Joint map " + get_name() + ": ";
  for(const std::string* kind : {&x_kind_, &y_kind_}) {
    if(*kind != "coordinate" && *kind != "rate") {
      throw shift::core::Exception(prefix + "x and y must be \"coordinate\" "
                                            "or \"rate\", not \"" + *kind +
                                   "\"");
    }
  }
  if(!map_file_.empty()) {
    if(!x_values_.empty() || !y_values_.empty() || !forces_.empty()) {
      throw shift::core::Exception(prefix + "give map_file or the values, "
                                            "not both");
    }
    const std::string path = context_->FindFile(map_file_);
    std::ifstream in(path);
    if(path.empty() || !in) {
      throw shift::core::Exception(prefix + "could not open " + map_file_);
    }
    std::stringstream text;
    text << in.rdbuf();
    std::string s = text.str();
    std::replace(s.begin(), s.end(), ',', ' ');
    std::istringstream numbers(s);
    size_t nx = 0, ny = 0;
    numbers >> nx >> ny;
    x_values_.resize(nx);
    y_values_.resize(ny);
    forces_.resize(nx * ny);
    for(auto* values : {&x_values_, &y_values_, &forces_}) {
      for(double& value : *values) {
        numbers >> value;
      }
    }
    if(!numbers) {
      throw shift::core::Exception(prefix + map_file_ + " is too short or "
                                                        "holds a value that "
                                                        "is not a number");
    }
  }
  if(x_values_.size() < 2 || y_values_.size() < 2 ||
     forces_.size() != x_values_.size() * y_values_.size()) {
    throw shift::core::Exception(prefix + "needs at least two x_values and "
                                          "two y_values, and a force for "
                                          "each pair of them");
  }
  for(const auto* values : {&x_values_, &y_values_}) {
    if(!std::is_sorted(values->begin(), values->end(),
                       std::less_equal<double>())) {
      throw shift::core::Exception(prefix + "x_values and y_values must "
                                            "increase");
    }
  }
  rows_.clear();
  const size_t nx = x_values_.size();
  for(size_t r = 0; r < y_values_.size(); r++) {
    std::vector<double> row(forces_.begin() + r * nx,
                            forces_.begin() + (r + 1) * nx);
    rows_.emplace_back(x_values_, row, Table::Extrapolation::kLinear);
  }
}

double shift::core::force_module::JointMap::Force(double x, double y) const {
  std::vector<double> column(rows_.size());
  for(size_t r = 0; r < rows_.size(); r++) {
    column[r] = rows_[r].Evaluate(x);
  }
  return Table(y_values_, column, Table::Extrapolation::kLinear).Evaluate(y);
}

double shift::core::force_module::JointMap::Variable(
  const std::string& kind, const joint::Base* joint, const double* y) const {
  return kind == "rate" ? y[context_->get_q_count() + joint->get_dq_index()]
                        : y[joint->get_q_index()];
}

void shift::core::force_module::JointMap::Service(const double* y, double*) {
  force_ = Force(Variable(x_kind_, joint_, y), Variable(y_kind_, y_joint_, y));
  context_->get_joint_working_forces()[joint_->get_dq_index()] += force_;
}

void shift::core::force_module::JointMap::UpdateOutputs(double,
                                                        const double* y) {
  force_ = Force(Variable(x_kind_, joint_, y), Variable(y_kind_, y_joint_, y));
}
