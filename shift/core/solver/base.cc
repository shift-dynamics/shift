#include "shift/core/solver/base.h"

#include <algorithm>

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/integrator/base.h"
#include "shift/core/solver/aba.h"
#include "shift/core/solver/auto.h"
#include "shift/core/solver/dca.h"
#include "shift/core/solver/rnca.h"

bool shift::core::solver::Modules::is_initialized() {
  if(!shift::core::solver::RNCA::is_registered()) {
    return false;
  }
  if(!shift::core::solver::ABA::is_registered()) {
    return false;
  }
  if(!shift::core::solver::DCA::is_registered()) {
    return false;
  }
  if(!shift::core::solver::Auto::is_registered()) {
    return false;
  }
  return true;
}

void shift::core::solver::Base::DeclareParameters() {
  DeclareParameter("type", &type_, "Solver type", true);
}

void shift::core::solver::Base::Load() {}

double shift::core::solver::Base::ElasticKineticEnergy(
  const topology::DHVA& dhva, const std::vector<int>* entries) const {
  double sum = 0.0;
  /* the rates the DHVA was evaluated at, with its velocities */
  if(dhva.dq.empty()) {
    return 0.0;
  }
  const double* dq = dhva.dq.data();
  const auto& mass_be = context_->get_elastic_mass_be();
  for(int i : context_->get_elastic_bodies()) {
    const auto body = context_->get_active_articulated_bodies()[i];
    if(entries && std::find(entries->begin(), entries->end(),
                            body->get_va_index()) == entries->end()) {
      continue;
    }
    const int n = (int)body->get_elastic_count();
    const int e = body->get_elastic_dq_index();
    const double* mass_ee = body->get_mass_ee();
    const ksl_screw_t& v = dhva.v[body->get_va_index()];
    for(int m = 0; m < n; m++) {
      sum += dq[e + m] * ksl_dot_cs(&mass_be[e + m], &v);
      for(int l = 0; l < n; l++) {
        sum += 0.5 * dq[e + l] * mass_ee[l * n + m] * dq[e + m];
      }
    }
  }
  return sum;
}

void shift::core::solver::Base::Connect() {
  // @TODO in the future, may allow multiple integrators and multiple solvers,
  // each handling a different part of the model, or allow loading multiple
  // integrators / solvers, with only one being active at a time. For now, hook
  // solver up to first integrator
  integrator_ = (*context_)["model_parameters"]["integrators"][0]
                  .get_data<shift::core::integrator::Base>();
}

void shift::core::solver::Base::ActivateObjects(bool parent_is_active) {
  assert(context_);
  context_->ActivateObject(this, parent_is_active && active_);
}
