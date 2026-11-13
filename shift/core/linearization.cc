#include "shift/core/linearization.h"

#include <lapacke.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <sstream>

#include "shift/core/body/free.h"
#include "shift/core/context.h"
#include "shift/core/controller/base.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/base.h"
#include "shift/core/integrator/base.h"
#include "shift/core/joint/base.h"
#include "shift/core/solver/base.h"

shift::core::Linearization::Linearization(shift::core::Context& context)
  : context_(context) {}

void shift::core::Linearization::NameStates() {
  std::vector<std::string> coordinates;
  for(auto joint : context_.get_active_joints()) {
    for(size_t i = 0; i < joint->get_dq_count(); i++) {
      coordinates.push_back(joint->FullyQualifiedName(joint->get_name()) +
                            "__dq_" + std::to_string(i));
    }
  }
  if(coordinates.size() != dq_count_) {
    throw shift::core::Exception(
      "Linearization: the joints have " + std::to_string(coordinates.size()) +
      " velocities, the model " + std::to_string(dq_count_));
  }
  std::vector<std::string> others;
  for(auto body : context_.get_free_bodies()) {
    for(int i = 0; i < shift::core::body::Free::kStateCount; i++) {
      others.push_back(body->FullyQualifiedName(body->get_name()) +
                       "__state_" + std::to_string(i));
    }
  }
  for(auto module : context_.get_active_force_modules()) {
    for(unsigned int i = 0; i < module->get_equation_count(); i++) {
      others.push_back(module->FullyQualifiedName(module->get_name()) +
                       "__state_" + std::to_string(i));
    }
  }
  for(auto controller : context_.get_active_controllers()) {
    for(unsigned int i = 0; i < controller->get_equation_count(); i++) {
      others.push_back(controller->FullyQualifiedName(controller->get_name()) +
                       "__state_" + std::to_string(i));
    }
  }
  if(others.size() != other_count_) {
    throw shift::core::Exception(
      "Linearization: the free bodies, force modules and controllers have " +
      std::to_string(others.size()) + " states, the model " +
      std::to_string(other_count_));
  }
  state_names_.clear();
  for(const auto& name : coordinates) {
    state_names_.push_back(name + " (displacement)");
  }
  for(const auto& name : coordinates) {
    state_names_.push_back(name + " (velocity)");
  }
  state_names_.insert(state_names_.end(), others.begin(), others.end());
  participant_names_ = coordinates;
  participant_names_.insert(participant_names_.end(), others.begin(),
                            others.end());
}

void shift::core::Linearization::Evaluate(double t,
                                          const std::vector<double>& y,
                                          std::vector<double>& dy) {
  if(context_.get_active_solvers().empty()) {
    throw shift::core::Exception("Linearization: the model has no solver");
  }
  if(context_.get_active_solvers()[0]->Solve(t, y, dy)) {
    throw shift::core::Exception(
      "Linearization: the solver failed at a perturbed state; try a smaller "
      "step");
  }
}

void shift::core::Linearization::Compute(double t,
                                         const std::vector<double>& y,
                                         double step) {
  q_count_ = context_.get_q_count();
  dq_count_ = context_.get_dq_count();
  const unsigned int y_size = static_cast<unsigned int>(y.size());
  if(y_size != context_.get_state_count() +
                 context_.get_auxiliary_state_count()) {
    throw shift::core::Exception("Linearization: the state has " +
                                 std::to_string(y_size) +
                                 " values, the model " +
                                 std::to_string(context_.get_state_count() +
                                   context_.get_auxiliary_state_count()));
  }
  other_count_ = y_size - q_count_ - dq_count_;
  size_ = 2 * dq_count_ + other_count_;
  NameStates();

  /* the constraints hold at the acceleration level only (see the class) */
  if(context_.get_active_integrators().empty()) {
    throw shift::core::Exception("Linearization: the model has no integrator");
  }
  shift::core::integrator::Base* integrator =
    context_.get_active_integrators()[0];
  const double omega_n = integrator->get_baumgarte_omega_n();
  integrator->set_baumgarte_omega_n(0.0);

  a_.assign(size_t(size_) * size_, 0.0);
  /* the displacements change at the velocities */
  for(unsigned int j = 0; j < dq_count_; j++) {
    a_[size_t(j) * size_ + dq_count_ + j] = 1.0;
  }

  std::vector<double> dy(y.size()), plus(y.size()), minus(y.size());
  std::vector<double> direction(y.size()), rates(y.size());
  try {
    for(unsigned int column = 0; column < size_; column++) {
      /* the change of y along the state of this column, per unit */
      std::fill(direction.begin(), direction.end(), 0.0);
      double scale = 1.0;
      if(column < dq_count_) {
        /* a displacement: the change of q at a unit velocity of this
        coordinate (quaternions by their rate, see ConvertdqTody) */
        std::vector<double> unit(y);
        std::fill(unit.begin() + q_count_, unit.begin() + q_count_ + dq_count_,
                  0.0);
        unit[q_count_ + column] = 1.0;
        context_.ConvertdqTody(unit, rates);
        std::copy(rates.begin(), rates.begin() + q_count_, direction.begin());
      } else if(column < 2 * dq_count_) {
        const unsigned int k = q_count_ + column - dq_count_;
        direction[k] = 1.0;
        scale = std::max(1.0, std::fabs(y[k]));
      } else {
        const unsigned int k = q_count_ + dq_count_ + column - 2 * dq_count_;
        direction[k] = 1.0;
        scale = std::max(1.0, std::fabs(y[k]));
      }
      const double h = step * scale;
      for(size_t k = 0; k < y.size(); k++) {
        plus[k] = y[k] + h * direction[k];
        minus[k] = y[k] - h * direction[k];
      }
      Evaluate(t, plus, dy);
      std::vector<double> dy_plus(dy);
      Evaluate(t, minus, dy);
      /* the rows of the velocities and of the other states */
      for(unsigned int row = dq_count_; row < size_; row++) {
        const unsigned int k = row < 2 * dq_count_
                                 ? q_count_ + row - dq_count_
                                 : q_count_ + dq_count_ + row - 2 * dq_count_;
        a_[size_t(row) * size_ + column] = (dy_plus[k] - dy[k]) / (2.0 * h);
      }
    }
    /* back to the state itself, so that the context and the modules hold
    what they held before */
    Evaluate(t, y, dy);
  } catch(...) {
    integrator->set_baumgarte_omega_n(omega_n);
    throw;
  }
  integrator->set_baumgarte_omega_n(omega_n);
}

std::vector<shift::core::Linearization::Mode>
shift::core::Linearization::Modes() const {
  const int n = static_cast<int>(size_);
  std::vector<Mode> modes;
  if(n == 0) {
    return modes;
  }
  std::vector<double> a(a_), wr(n), wi(n), vl(size_t(n) * n),
    vr(size_t(n) * n);
  const int info = LAPACKE_dgeev(LAPACK_ROW_MAJOR, 'V', 'V', n, a.data(), n,
                                 wr.data(), wi.data(), vl.data(), n, vr.data(),
                                 n);
  if(info != 0) {
    throw shift::core::Exception("Linearization: the eigenvalues did not "
                                 "converge (dgeev " + std::to_string(info) +
                                 ")");
  }
  const unsigned int participants =
    static_cast<unsigned int>(participant_names_.size());
  for(int i = 0; i < n; i++) {
    if(wi[i] < 0.0) {
      continue; // the second of a pair, as its first
    }
    /* columns i (real) or i and i + 1 (real and imaginary parts) */
    std::vector<std::complex<double>> v(n), w(n);
    for(int k = 0; k < n; k++) {
      if(wi[i] > 0.0) {
        v[k] = {vr[size_t(k) * n + i], vr[size_t(k) * n + i + 1]};
        w[k] = {vl[size_t(k) * n + i], vl[size_t(k) * n + i + 1]};
      } else {
        v[k] = vr[size_t(k) * n + i];
        w[k] = vl[size_t(k) * n + i];
      }
    }
    /* dgeev's left eigenvectors satisfy u^H A = lambda u^H: the
    participation of state k is |conj(u_k) v_k| / |u^H v| */
    std::complex<double> norm = 0.0;
    for(int k = 0; k < n; k++) {
      norm += std::conj(w[k]) * v[k];
    }
    Mode mode;
    mode.eigenvalue = {wr[i], wi[i]};
    mode.participation.assign(participants, 0.0);
    for(int k = 0; k < n; k++) {
      const double p = std::abs(std::conj(w[k]) * v[k]) /
                       std::max(std::abs(norm), 1e-300);
      const unsigned int k_unsigned = static_cast<unsigned int>(k);
      const unsigned int participant =
        k_unsigned < dq_count_       ? k_unsigned
        : k_unsigned < 2 * dq_count_ ? k_unsigned - dq_count_
                                     : k_unsigned - dq_count_;
      mode.participation[participant] += p;
    }
    modes.push_back(std::move(mode));
  }
  std::stable_sort(modes.begin(), modes.end(),
                   [](const Mode& a, const Mode& b) {
                     return std::abs(a.eigenvalue) > std::abs(b.eigenvalue);
                   });
  return modes;
}

std::string shift::core::Linearization::Report(const std::vector<Mode>& modes,
                                               int count) const {
  std::ostringstream out;
  char line[256];
  std::snprintf(line, sizeof(line), "%4s %13s %13s %11s %9s %9s %10s\n",
                "mode", "real (1/s)", "imag (rad/s)", "|lambda|", "freq (Hz)",
                "damping", "h (ms)");
  out << line;
  for(int i = 0; i < std::min<int>(count, static_cast<int>(modes.size()));
      i++) {
    const auto& mode = modes[i];
    const double size = std::abs(mode.eigenvalue);
    const double frequency = mode.eigenvalue.imag() / (2.0 * M_PI);
    const double damping =
      size > 0.0 ? -mode.eigenvalue.real() / size : 0.0;
    std::snprintf(line, sizeof(line),
                  "%4d %13.4g %13.4g %11.4g %9.4g %9.3f %10.4g\n", i + 1,
                  mode.eigenvalue.real(), mode.eigenvalue.imag(), size,
                  frequency, damping, size > 0.0 ? 1e3 / size : 0.0);
    out << line;
    std::vector<size_t> order(mode.participation.size());
    std::iota(order.begin(), order.end(), size_t(0));
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
      return mode.participation[a] > mode.participation[b];
    });
    for(size_t k = 0; k < std::min<size_t>(5, order.size()); k++) {
      if(mode.participation[order[k]] < 0.02) {
        break;
      }
      std::snprintf(line, sizeof(line), "       %5.1f%%  %s\n",
                    100.0 * mode.participation[order[k]],
                    participant_names_[order[k]].c_str());
      out << line;
    }
  }
  return out.str();
}
