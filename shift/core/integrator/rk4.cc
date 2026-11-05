#include "shift/core/integrator/rk4.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/log.h"

#include <math.h>
#include <sstream>

bool shift::core::integrator::RK4::registered_ =
  shift::core::Factory<shift::core::integrator::Base>::get()->RegisterModule(
    "rk4", shift::core::integrator::RK4::Create);

std::unique_ptr<shift::core::integrator::RK4>
shift::core::integrator::RK4::Create() {
  return std::make_unique<shift::core::integrator::RK4>();
}

void shift::core::integrator::RK4::DeclareParameters() {
  DeclareParameter(
    "step_size", &step_size_,
    "fixed internal step size used for numerical integration, defaults to "
    "time_step if not provided. Results are reported every time_step.",
    false);
  DeclareParameter("integral_gain", &integral_gain_,
                   "integral gain used in baumgarte constraint stabilization, "
                   "defaults to 0.0 (off) if not provided",
                   false);
}

void shift::core::integrator::RK4::Connect() {
  assert(context_);
  solver_ = (*context_)["model_parameters"]["solvers"][0]
              .get_data<shift::core::solver::Base>();
  assert(solver_);
}

void shift::core::integrator::RK4::Reset() {

  neqn_ = context_->get_q_count() + context_->get_dq_count() +
          context_->get_force_module_equation_count();
  y_.resize(neqn_);
  std::fill(y_.begin(), y_.end(), 0.0);

  /* copy initial conditions from context */
  memcpy(y_.data(), context_->get_q().data(),
         (context_->get_q_count() + context_->get_dq_count()) * sizeof(double));

  /* copy initial conditions from force modules */
  int i = context_->get_q_count() + context_->get_dq_count();
  for(auto& force_module : context_->get_active_force_modules()) {
    for(unsigned int k = 0; k < force_module->get_equation_count(); k++, i++) {
      y_[i] = force_module->get_q0()[k];
    }
  }

  integral_error_.resize(context_->get_constraint_equation_count());
  std::fill(integral_error_.begin(), integral_error_.end(), 0.0);

  for(auto* v : {&k1_, &k2_, &k3_, &k4_, &yt_}) {
    v->resize(neqn_);
    std::fill(v->begin(), v->end(), 0.0);
  }

  t_ = 0.0;
  h_ = step_size_ > 0.0 ? step_size_ : time_step_;

  baumgarte_omega_n_ = omega_n_;
  baumgarte_omega_n_prev_ = omega_n_;
}

void shift::core::integrator::RK4::Initialize() {
  Reset();
}

void shift::core::integrator::RK4::StepFunction(const double t,
                                                const std::vector<double>& y,
                                                std::vector<double>& dy) {
  assert(solver_);
  if(solver_->Solve(t, y, dy)) {
    std::stringstream ss;
    ss << "Newton-Raphson failure at t = " << t
       << ", integration stopped. Last successfully integrated time: " << t_;
    SHIFT_ERROR(ss.str());
    throw shift::core::Exception(ss.str());
  }
}

/*!
@brief takes one classical fourth-order Runge-Kutta step of size h from
(t_, y_), where k1_ already holds the state derivatives at (t_, y_). On return
y_ and t_ are advanced and k1_ holds the state derivatives at the new states,
which also leaves the context (ddq, constraint errors) consistent with y_ for
logging.
*/
void shift::core::integrator::RK4::Step(const double h) {

  for(size_t i = 0; i < neqn_; i++) {
    yt_[i] = y_[i] + 0.5 * h * k1_[i];
  }
  StepFunction(t_ + 0.5 * h, yt_, k2_);

  for(size_t i = 0; i < neqn_; i++) {
    yt_[i] = y_[i] + 0.5 * h * k2_[i];
  }
  StepFunction(t_ + 0.5 * h, yt_, k3_);

  for(size_t i = 0; i < neqn_; i++) {
    yt_[i] = y_[i] + h * k3_[i];
  }
  StepFunction(t_ + h, yt_, k4_);

  for(size_t i = 0; i < neqn_; i++) {
    y_[i] += h * (k1_[i] + 2.0 * k2_[i] + 2.0 * k3_[i] + k4_[i]) / 6.0;
  }
  t_ += h;

  StepFunction(t_, y_, k1_);

  // evaluate integral error
  const std::vector<double>& e = context_->get_jacobian().get_constraints();
  const double rho_dt = h * integral_gain_;
  for(size_t i = 0; i < e.size(); i++) {
    integral_error_[i] += e[i] * rho_dt;
  }
}

void shift::core::integrator::RK4::Integrate() {

  Reset();
  next_progress_percent_ = 10;
  solver_->Initialize();

  if(neqn_ < 1 || time_span_ <= 0.0 || time_step_ <= 0.0 || h_ <= 0.0) {
    std::stringstream ss;
    ss << "Improper simulation parameters detected: neqn = " << neqn_
       << ", time_span = " << time_span_ << ", time_step = " << time_step_
       << ", step_size = " << h_;
    SHIFT_ERROR(ss.str());
    throw shift::core::Exception(ss.str());
  }

  StepFunction(t_, y_, k1_);
  context_->WriteStates(t_, y_);

  /* report every time_step_, taking as many internal steps of at most h_ as
  needed to land exactly on each reporting time */
  const unsigned int report_count = ceil(time_span_ / time_step_ - 1e-9);
  for(unsigned int report = 1; report <= report_count; report++) {
    const double t_report = fmin(report * time_step_, time_span_);
    while(t_report - t_ > 1e-12 * fmax(1.0, t_report)) {
      Step(fmin(h_, t_report - t_));
    }
    context_->WriteStates(t_, y_);
    ReportProgress(t_);
  }

  SHIFT_INFO("Simulation finished t=" + std::to_string(t_));
}

void shift::core::integrator::RK4::get_current_time(double& time,
                                                    double& delta_time) {
  time = t_;
  delta_time = h_;
}
