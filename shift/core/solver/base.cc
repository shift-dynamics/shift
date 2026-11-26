#include "shift/core/solver/base.h"

#include <algorithm>

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/integrator/base.h"
#include "shift/core/solver/aba.h"
#include "shift/core/solver/auto.h"
#include "shift/core/solver/dca.h"
#include "shift/core/solver/rnca.h"
#include "shift/core/tangents.h"

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

void shift::core::solver::Base::ForEachTangentEntry(
  const Context& context, const Tangents& tangents, double c, double k,
  const std::function<void(int row, int column, double value)>& add) {
  for(const auto& entry : tangents.get_joint_stiffness()) {
    add(entry.dq_a, entry.dq_b, k * entry.value);
  }
  for(const auto& entry : tangents.get_joint_damping()) {
    add(entry.dq_a, entry.dq_b, c * entry.value);
  }
  std::vector<int> path_a;
  std::vector<int> path_b;
  std::vector<ksl_screw_t> screws_a;
  std::vector<ksl_screw_t> screws_b;
  auto add_blocks = [&](const std::vector<Tangents::BodyBlock>& blocks,
                        double scale) {
    if(scale == 0.0) {
      return;
    }
    for(const auto& body_block : blocks) {
      context.GetLoadPath(body_block.load_index_a, path_a, screws_a);
      context.GetLoadPath(body_block.load_index_b, path_b, screws_b);
      for(size_t m_a = 0; m_a < path_a.size(); m_a++) {
        /* w = K_ab^T h_l, so that h_l^T K_ab h_k = w . h_k */
        double w[6];
        for(int col = 0; col < 6; col++) {
          w[col] = 0.0;
          for(int row = 0; row < 6; row++) {
            w[col] += screws_a[m_a].at[row] * body_block.block[row * 6 + col];
          }
        }
        for(size_t m_b = 0; m_b < path_b.size(); m_b++) {
          double value = 0.0;
          for(int m = 0; m < 6; m++) {
            value += w[m] * screws_b[m_b].at[m];
          }
          add(path_a[m_a], path_b[m_b], scale * value);
        }
      }
    }
  };
  add_blocks(tangents.get_body_stiffness(), k);
  add_blocks(tangents.get_body_damping(), c);
}
