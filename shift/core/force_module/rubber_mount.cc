#include "shift/core/force_module/rubber_mount.h"

#include "shift/core/exception.h"
#include "shift/core/factory.h"

bool shift::core::force_module::RubberMount::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "rubber_mount", shift::core::force_module::RubberMount::Create);

std::unique_ptr<shift::core::force_module::RubberMount>
shift::core::force_module::RubberMount::Create() {
  return std::make_unique<shift::core::force_module::RubberMount>();
}

void shift::core::force_module::RubberMount::DeclareMountParameters() {
  DeclareParameter("cubic_stiffness", &cubic_stiffness_,
                   "6 m numbers, hardening stiffness k3 of each coordinate; "
                   "optional, default zero",
                   false)
    .set_row_dim(-1);
  DeclareParameter("maxwell_stiffness", &maxwell_stiffness_,
                   "6 m numbers per Maxwell element, element by element: "
                   "the spring stiffness of each coordinate; optional",
                   false)
    .set_row_dim(-1);
  DeclareParameter("maxwell_damping", &maxwell_damping_,
                   "6 m numbers per Maxwell element, as maxwell_stiffness: "
                   "the damper coefficient in series with each spring",
                   false)
    .set_row_dim(-1);
}

void shift::core::force_module::RubberMount::Load() {
  Bushing::Load();
  const std::string prefix = "Rubber mount " + get_name() + ": ";
  const size_t n = 6 * pairs_.size();
  if(!cubic_stiffness_.empty() && cubic_stiffness_.size() != n) {
    throw shift::core::Exception(prefix + "cubic_stiffness needs " +
                                 std::to_string(n) + " numbers");
  }
  if(cubic_stiffness_.empty()) {
    cubic_stiffness_.assign(n, 0.0);
  }
  if(maxwell_stiffness_.size() != maxwell_damping_.size() ||
     maxwell_stiffness_.size() % n != 0) {
    throw shift::core::Exception(
      prefix + "maxwell_stiffness and maxwell_damping need " +
      std::to_string(n) + " numbers for each element, as many of each");
  }
  elements_.clear();
  for(size_t k = 0; k < maxwell_stiffness_.size(); k++) {
    const double stiffness = maxwell_stiffness_[k];
    const double damping = maxwell_damping_[k];
    if(stiffness == 0.0) {
      continue;
    }
    if(!(stiffness > 0.0) || !(damping > 0.0)) {
      throw shift::core::Exception(
        prefix + "each Maxwell element needs positive stiffness and "
        "damping, or zero stiffness for none");
    }
    Element element;
    element.coordinate = k % n;
    element.stiffness = stiffness;
    element.rate = stiffness / damping;
    elements_.push_back(element);
  }
  equation_count_ = elements_.size();
  q_0_.assign(equation_count_, 0.0);
}

void shift::core::force_module::RubberMount::AddForces(const double* y,
                                                       double* dy) {
  for(size_t c = 0; c < x_.size(); c++) {
    const double x = x_[c];
    f_[c] -= cubic_stiffness_[c] * x * x * x;
  }
  for(size_t e = 0; e < elements_.size(); e++) {
    const Element& element = elements_[e];
    const double stretch = y[state_offset_ + e];
    f_[element.coordinate] -= element.stiffness * stretch;
    if(dy) {
      dy[state_offset_ + e] = v_[element.coordinate] - element.rate * stretch;
    }
  }
}
