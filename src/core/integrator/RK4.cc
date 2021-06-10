#include "shift/core/integrator/RK4.h"
#include "shift/core/Context.h"
#include "shift/core/Factory.h"

#include <math.h>

bool shift::core::integrator::RK4::registered_ =
  shift::core::Factory<shift::core::integrator::Base>::get()->RegisterModule(
    "rk4", shift::core::integrator::RK4::create);

std::unique_ptr<shift::core::integrator::RK4> shift::core::integrator::RK4::create() {
  return std::make_unique<shift::core::integrator::RK4>();
}

void shift::core::integrator::RK4::DeclareParameters() {}

void shift::core::integrator::RK4::connect() {
  assert(context_);
  solver_ =
    (*context_)["model_parameters"]["solvers"][0].data<shift::core::solver::Base>();
  assert(solver_);
}

void shift::core::integrator::RK4::reset() {

  assert(context_);
  neqn_ = context_->qCount() + context_->dqCount() +
          context_->forceModuleEquationCount();
  y_.resize(neqn_);
  std::fill(y_.begin(), y_.end(), 0);

  /* copy initial conditions from context */
  memcpy(y_.data(), context_->q().data(),
         (context_->qCount() + context_->dqCount()) * sizeof(double));

  /* copy initial conditions from force modules */
  int i = context_->qCount() + context_->dqCount();
  for(auto& force_module : context_->activeforcemodules()) {
    for(unsigned int k = 0; k < force_module->equationCount(); k++, i++) {
      y_[i] = force_module->q0()[k];
    }
  }

  integral_error_.resize(context_->chordConstraintCount() * 6);
  std::fill(integral_error_.begin(), integral_error_.end(), 0);

  step_count_ = ceil(time_span_ / time_step_) + 1;

  f0_.resize(neqn_);
  f1_.resize(neqn_);
  f2_.resize(neqn_);
  f3_.resize(neqn_);

  u0_.resize(neqn_);
  u1_.resize(neqn_);
  u2_.resize(neqn_);
  u3_.resize(neqn_);

  std::fill(f0_.begin(), f0_.end(), 0);
  std::fill(f1_.begin(), f1_.end(), 0);
  std::fill(f2_.begin(), f2_.end(), 0);
  std::fill(f3_.begin(), f3_.end(), 0);

  std::fill(u0_.begin(), u0_.end(), 0);
  std::fill(u1_.begin(), u1_.end(), 0);
  std::fill(u2_.begin(), u2_.end(), 0);
  std::fill(u3_.begin(), u3_.end(), 0);
}

void shift::core::integrator::RK4::initialize() {
  reset();
}

void shift::core::integrator::RK4::integrate() {
  reset();
  solver_->initialize();

  unsigned int step = 0;
  unsigned int steps = ceil(time_span_ / time_step_);
  while(step <= steps) {

    step++;
  }
}

/*! @brief RK4VEC takes one Runge-Kutta step for a vector ODE
//
//  Purpose:
//
//    .
//
//  Discussion:
//
//    It is assumed that an initial value problem, of the form
//
//      du/dt = f ( t, u )
//      u(t0) = u0
//
//    is being solved.
//
//    If the user can supply current values of t, u, a stepsize dt, and a
//    function to evaluate the derivative, this function can compute the
//    fourth-order Runge Kutta estimate to the solution at time t+dt.
//
//  Licensing:
//
//    This code is distributed under the GNU LGPL license.
//
//  Modified:
//
//    09 October 2013
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, double T0, the current time.
//
//    Input, int M, the spatial dimension.
//
//    Input, double U0[M], the solution estimate at the current time.
//
//    Input, double DT, the time step.
//
//    Input, double *F ( double T, int M, double U[] ), a function which
//    evaluates the derivative, or right hand side of the problem.
//
//    Output, double RK4VEC[M], the fourth-order Runge-Kutta solution estimate
//    at time T0+DT.
*/
void shift::core::integrator::RK4::step() {

  assert(solver_);
  solver_->solve(y_.data(), dy_.data());

  //  Get four sample values of the derivative.
  f0 = f(t0, m, u0);

  double t1 = time_step_ / 2.0;
  u1 = new double[m];
  for(size_t i = 0; i < neqn_; i++) {
    u1[i] = u0[i] + time_step_ * f0[i] / 2.0;
  }
  f1 = f(t1, m, u1);

  t2 = t0 + dt / 2.0;
  u2 = new double[m];
  for(i = 0; i < m; i++) {
    u2[i] = u0[i] + time_step_ * f1[i] / 2.0;
  }
  f2 = f(t2, m, u2);

  t3 = t0 + dt;
  u3 = new double[m];
  for(i = 0; i < m; i++) {
    u3[i] = u0[i] + time_step_ * f2[i];
  }
  f3 = f(t3, m, u3);

  //  Combine them to estimate the solution.
  u = new double[m];
  for(i = 0; i < m; i++) {
    u[i] = u0[i] + dt * (f0[i] + 2.0 * f1[i] + 2.0 * f2[i] + f3[i]) / 6.0;
  }

  // std::cout << "y: ";
  // for(size_t i = 0; i < neqn_; i++) {
  //   std::cout << y[i] << " ";
  // }
  // std::cout << std::endl;
  //
  // std::cout << "dy: ";
  // for(size_t i = 0; i < dy_.size(); i++) {
  //   std::cout << dy_[i] << " ";
  // }
  // std::cout << std::endl << std::endl;

  if(iconv_ == 1) {
    auto& jacobian = context_->jacobian();
    auto& cs = jacobian.constraints();

    h_ = fmax(x_ - t_, hmin_); /* limit next step size */
    double omega_n = stability_limits_[k_] / h_ * safety_factor_;
    omega_n = fmin(omega_n, omega_n_max_); /* limit omega_n */
    omega_n = omega_n_prev_ + smoothing_filter_ * (omega_n - omega_n_prev_);
    omega_n_prev_ = omega_n;
    baumgarte_2_omega_n_ = 2.0 * omega_n;
    baumgarte_omega_n_2_ = omega_n * omega_n;
    double rho_dt = h_ * integral_gain_;
    for(unsigned int i = 0; i < context_->constraintEquationCount(); i++) {
      // evaluate integral error
      integral_error_[i] += cs[i] * rho_dt;
    }

    t_ = x_;
    if(x_ >= tdata_[step_] + time_step_) {
      // do logging
      context_->writeStates(t_, y);
      iteration_count_ = 0;
    }
  }
  iconv_ = 0;

  double* f0;
  double* f1;
  double* f2;
  double* f3;
  int i;
  double t1;
  double t2;
  double t3;
  double* u;
  double* u1;
  double* u2;
  double* u3;

  step(const double y[], double dy[]) {

    //  Get four sample values of the derivative.
    f0 = f(t0, m, u0);

    t1 = t0 + dt / 2.0;
    u1 = new double[m];
    for(i = 0; i < m; i++) {
      u1[i] = u0[i] + dt * f0[i] / 2.0;
    }
    f1 = f(t1, m, u1);

    t2 = t0 + dt / 2.0;
    u2 = new double[m];
    for(i = 0; i < m; i++) {
      u2[i] = u0[i] + dt * f1[i] / 2.0;
    }
    f2 = f(t2, m, u2);

    t3 = t0 + dt;
    u3 = new double[m];
    for(i = 0; i < m; i++) {
      u3[i] = u0[i] + dt * f2[i];
    }
    f3 = f(t3, m, u3);

    //  Combine them to estimate the solution.
    u = new double[m];
    for(i = 0; i < m; i++) {
      u[i] = u0[i] + dt * (f0[i] + 2.0 * f1[i] + 2.0 * f2[i] + f3[i]) / 6.0;
    }

    //  Free memory.
    delete[] f0;
    delete[] f1;
    delete[] f2;
    delete[] f3;
    delete[] u1;
    delete[] u2;
    delete[] u3;

    return u;
  }
