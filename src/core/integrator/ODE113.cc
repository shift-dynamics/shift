#include "shift/core/integrator/ODE113.h"
#include "shift/core/Context.h"
#include "shift/core/Factory.h"
#include "shift/core/log.h"

#include <math.h>

const double shift::core::integrator::ODE113::twou_ = 1.42e-14;
// std::numeric_limits<double>::epsilon() * 2.0;

const double shift::core::integrator::ODE113::fouru_ = 2.84e-14;
// std::numeric_limits<double>::epsilon() * 4.0;

const double shift::core::integrator::ODE113::gstr_[14] = {
  0.0,    0.5,     0.0833,  0.0417,  0.0264,  0.0188,  0.0143,
  0.0114, 0.00936, 0.00789, 0.00679, 0.00592, 0.00524, 0.00468};

const double shift::core::integrator::ODE113::two_[14] = {
  0.0,   2.0,   4.0,   8.0,    16.0,   32.0,   64.0,
  128.0, 256.0, 512.0, 1024.0, 2048.0, 4096.0, 8192.0};

const double shift::core::integrator::ODE113::stability_limits_[17] = {
  1.0,  2.0,  2.26, 1.86, 1.4, 1.03, 0.78, 0.58, 0.44,
  0.34, 0.26, 0.21, 0.6,  0.2, 0.2,  0.2,  0.2};

bool shift::core::integrator::ODE113::registered_ =
  shift::core::Factory<shift::core::integrator::Base>::get()->RegisterModule(
    "ode113", shift::core::integrator::ODE113::Create);

std::unique_ptr<shift::core::integrator::ODE113> shift::core::integrator::ODE113::Create() {
  return std::make_unique<shift::core::integrator::ODE113>();
}

void shift::core::integrator::ODE113::DeclareParameters() {
  DeclareParameter("safety_factor", &safety_factor_,
               "adjustable safety factor for numerical integration: defaults "
               "to 0.0005 if not provided",
               false);
  DeclareParameter("hmax", &hmax_,
               "largest internal time step allowed by numerical integrator, "
               "defaults to 0.01 if not provided.",
               false);
  DeclareParameter("omega_n_max", &omega_n_max_,
               "largest value for omega_n used in Baumgarte constraint "
               "stabilization, defaults to 1000.0 if not provided",
               false);
  DeclareParameter("integral_gain", &integral_gain_,
               "integral gain used in baumgarte constraint stabilization, "
               "defaults to 1000.0 if not provided",
               false);
  DeclareParameter("absolute_error", &absolute_error_,
               "absolute error used in numerical integration, defaults to "
               "0.0001 if not provided",
               false);
  DeclareParameter("relative_error", &relative_error_,
               "relative error used in numerical integration, defaults to 0.0 "
               "if not provided",
               false);
}

void shift::core::integrator::ODE113::Connect() {
  assert(context_);
  solver_ =
    (*context_)["model_parameters"]["solvers"][0].get_data<shift::core::solver::Base>();
  assert(solver_);
}

void shift::core::integrator::ODE113::Reset() {

  neqn_ = context_->get_q_count() + context_->get_dq_count() +
          context_->get_force_module_equation_count();
  y_.resize(neqn_);
  std::fill(y_.begin(), y_.end(), 0);

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
  std::fill(integral_error_.begin(), integral_error_.end(), 0);

  step_count_ = ceil(time_span_ / time_step_) + 1;

  yy_.resize(neqn_);
  std::fill(yy_.begin(), yy_.end(), 0);

  ydata_.resize(neqn_ * step_count_);
  std::fill(ydata_.begin(), ydata_.end(), 0);

  tdata_.resize(step_count_);
  std::fill(tdata_.begin(), tdata_.end(), 0);

  memcpy(ydata_.data(), y_.data(), neqn_ * sizeof(double));

  dy_.resize(neqn_);
  std::fill(dy_.begin(), dy_.end(), 0);

  yy_.resize(neqn_);
  std::fill(yy_.begin(), yy_.end(), 0);
  memcpy(yy_.data(), y_.data(), neqn_ * sizeof(double));

  // dyout_.resize(neqn_);
  // std::fill(dyout_.begin(), dyout_.end(), 0);

  wt_.resize(neqn_);
  std::fill(wt_.begin(), wt_.end(), 0);

  p_.resize(neqn_);
  std::fill(p_.begin(), p_.end(), 0);

  yerr_.resize(neqn_);
  std::fill(yerr_.begin(), yerr_.end(), 0);

  phi_ = shift::core::Array2d<double>(17, neqn_);
  phi_.zero();

  iteration_count_ = 0;

  memset(beta_, 0, 13 * sizeof(double));
  memset(psi_, 0, 13 * sizeof(double));
  memset(alpha_, 0, 13 * sizeof(double));
  memset(w_, 0, 14 * sizeof(double));
  memset(v_, 0, 13 * sizeof(double));
  memset(g_, 0, 14 * sizeof(double));
  g_[1] = 1.0;
  g_[2] = 0.5;
  memset(sig_, 0, 14 * sizeof(double));
  sig_[1] = 1.0;

  baumgarte_omega_n_ = omega_n_;
  baumgarte_omega_n_prev_ = omega_n_;
}

void shift::core::integrator::ODE113::Initialize() {
  Reset();
}

void shift::core::integrator::ODE113::StepFunction(double t, std::vector<double>& y) {
  assert(solver_);
  iteration_count_++;

  newton_raphson_failure_ = solver_->Solve(t, y, dy_);
  if(newton_raphson_failure_) {
    return;
  }

  if(iconv_ == 1) {

    const std::vector<double>& cs = context_->get_jacobian().get_constraints();

    double h = fmax(x_ - t_, hmin_); /* limit next step size */

    baumgarte_omega_n_ = stability_limits_[k_] / h * safety_factor_;
    baumgarte_omega_n_ =
      fmin(baumgarte_omega_n_, omega_n_max_); /* limit omega_n */
    baumgarte_omega_n_ =
      baumgarte_omega_n_prev_ +
      smoothing_filter_ * (baumgarte_omega_n_ - baumgarte_omega_n_prev_);
    baumgarte_omega_n_prev_ = baumgarte_omega_n_;

    double rho_dt = h * integral_gain_;

    // evaluate integral error
    for(unsigned int i = 0; i < context_->get_constraint_equation_count(); i++) {
      integral_error_[i] += cs[i] * rho_dt;
    }

    t_ = x_;

#if SHIFT_LOG_LEVEL > 4
    {
      std::stringstream ss;
      ss << "cs: ";
      for(auto& c : cs) {
        ss << c << " ";
      }
      SHIFT_TRACE(ss.str());
    }

    {
      std::stringstream ss;
      ss << "integral_error: "; 
      for(auto& i : integral_error_) {
        ss << i << " "; 
      }
      SHIFT_TRACE(ss.str());
    }

    {
      std::stringstream ss;
      ss << "(" << t_ << ") y: ";
      for(size_t i = 0; i < neqn_; i++) {
        ss << y_[i] << " ";
      }
      SHIFT_TRACE(ss.str());
    }

    {
      std::stringstream ss;
      ss << "(" << t_ << ") dy: ";
      for(size_t i = 0; i < neqn_; i++) {
        ss << dy_[i] << " ";
      }
      SHIFT_TRACE(ss.str());
    }
#endif

    if(x_ >= tdata_[step_] + time_step_) {
      // do logging
      context_->WriteStates(t_, y);
      iteration_count_ = 0;
    }
  }
  iconv_ = 0;
  return;
}

void shift::core::integrator::ODE113::Integrate() {

  Reset();
  solver_->Initialize();
  context_->WriteStates(x_, y_);

  step_ = 0;
  eps_ = fmax(relative_error_, absolute_error_);

  /* test for improper parameters */
  if(neqn_ < 1 || time_span_ <= 0.0 || relative_error_ < 0.0 ||
     absolute_error_ < 0.0 || iflag_ < 1 || iflag_ > 5 || eps_ < twou_) {

    std::cerr << "Improper simulation parameters detected:" << std::endl;
    fprintf(
      stderr,
      "neqn=%d, time=%g, time_end=%g, relerr=%g, abserr=%g, iflag=%d, eps=%g, "
      "twou=%g\n",
      neqn_, 0.0, time_span_, relative_error_, absolute_error_, iflag_, eps_,
      twou_);

    iflag_ = 6;
    return;
  }

  /*  on each call, set interval of integration and counter for number of
      steps.  adjust input error tolerances to define weight vector.   */
  double releps = relative_error_ / eps_;
  double abseps = absolute_error_ / eps_;

  /*  on start and restart, set work variables x and yy[], store the
      direction of integration and initialize the step size */
  if(iflag_ == 1) {
    start_ = true;
    k_ = 0;
    kold_ = 0;
    x_ = 0.0;
    memcpy(yy_.data(), y_.data(), neqn_ * sizeof(double));
    h_ = hmax_;
    for(int i = 0; i < 13; i++) {
      psi_[i] = 0;
    }
  }

  while(true) {
    /*  limit step size, set weight vector and take a step */
    h_ = fmin(h_, hmax_);
    h_ = fmin(h_, time_span_ - x_);
    for(size_t l = 0; l < neqn_; l++) {
      wt_[l] = releps * fabs(yy_[l]) + abseps;
    }

    step();

    if(newton_raphson_failure_) {
      SHIFT_ERROR("Newton-Raphson failure, integration stopped");
      return;
    }

    if(x_ >= time_span_) {
      iflag_ = 2;
      SHIFT_INFO("Simulation finished");
      return;
    }

    /*  If crash, increase error tolerances and return to
      calling program. Can call de again to continue integration with larger
      tolerances.*/
    if(crash_) {
      iflag_ = 3;
      relative_error_ = eps_ * releps;
      absolute_error_ = eps_ * abseps;
      memcpy(y_.data(), yy_.data(), neqn_ * sizeof(double));
      x_ = t_;
      return;
    } /* End if crash*/
  }   /* End While*/

  /*! if number of steps is less than total amount of space allocated, pad the
  states out to the end of the space allocated to avoid incorrect display in
  graphics

  @TODO need to add interpolation in graphics for data at irregular time
  interval reporting */
  if(step_ < step_count_ - 1) {
    for(unsigned int i = step_; i < step_count_; i++) {
      memcpy(ydata_.data() + neqn_ * i, ydata_.data() + neqn_ * (step_ - 1),
             neqn_ * sizeof(double));
    }
  }

  if(iflag_ == 3) {
    SHIFT_ERROR("ODE113 was unable to continue.");
  }

  if(newton_raphson_failure_) {
    SHIFT_ERROR("Newton Raphson failed, solver stopped");
  }
}

/*
@brief Subroutine step integrates a system of first order ordinary
differential equations one step, normally from x to x+h, using a
modified divided difference form of the adams pece formulas.

Local extrapolation is used to improve absolute stability and accuracy.
the code adjusts its order and step size to control the local error
per unit step in a generalized sense.  special devices are included
to control roundoff error and to detect when the user is requesting
too much accuracy.

This code is completely explained and documented in the text,
computer solution of ordinary differential equations:  the initial
value problem  by l. f. shampine and m. k. gordon.

The parameters represent:
    x -- independent variable
    y(*) -- solution vector at x
    yp(*) -- derivative of solution vector at  x  after successful
         step
    neqn -- number of equations to be integrated
    h -- appropriate step size for next step.  normally determined by
         code
    eps -- local error tolerance.  must be variable
    wt(*) -- vector of weights for error criterion
    start -- logical variable set .true. for first step,  .false.
         otherwise
    hold -- step size used for last successful step
    k -- appropriate order for next step (determined by code)
    kold -- order used for last successful step
    crash -- logical variable set .true. when no step can be taken,
         .false. otherwise.

The arrays  phi, psi  are required for the interpolation subroutine
intrp .  the array  pp  is internal to the code.

input to  step
    first call --

the user must provide storage in his driver program for all arrays
in the call list, namely
   real*8 y(dim ),wt(dim ),phi(dim ,16),pp(dim ),yp(dim ),psi(12)
the user must also declare  start  and  crash  logical variables
and  f  an external subroutine, supply the subroutine  f(x,y,yp)
to evaluate
    dy(i)/dx = yp(i) = f(x,y(1),y(2),...,y(neqn))

and initialize only the following parameters:
    x -- initial value of the independent variable
    y(*) -- vector of initial values of dependent variables
    neqn -- number of equations to be integrated
    h -- nominal step size indicating direction of integration
         and maximum size of step.  must be variable
    eps -- local error tolerance per step.  must be variable
    wt(*) -- vector of non-zero weights for error criterion
    start -- true.

step  requires the l2 norm of the vector with components
local error(l)/wt(l)  be less than  eps  for a successful step.  the
array  wt  allows the user to specify an error test appropriate
for his problem.  for example,
    wt(l) = 1.0  specifies absolute error,
          = abs(y(l))  error relative to the most recent value of the
               l-th component of the solution,
          = abs(yp(l))  error relative to the most recent value of
               the l-th component of the derivative,
          = amax1(wt(l),abs(y(l)))  error relative to the largest
               magnitude of l-th component obtained so far,
          = abs(y(l))*relerr/eps + abserr/eps  specifies a mixed
               relative-absolute test where  relerr  is relative
               error,  abserr  is absolute error and  eps =
               amax1(relerr,abserr) .

subsequent calls --

  subroutine  step  is designed so that all information needed to
  continue the integration, including the step size  h  and the order
  k , is returned with each step.  with the exception of the step
  size, the error tolerance, and the weights, none of the parameters
  should be altered.  the array  wt  must be updated after each step
  to maintain relative error tests like those above.  normally the
  integration is continued just beyond the desired endpoint and the
  solution interpolated there with subroutine  intrp .  if it is
  impossible to integrate beyond the endpoint, the step size may be
  reduced to hit the endpoint since the code will not take a step
  larger than the  h  input.  changing the direction of integration,
  i.e., the sign of  h , requires the user set  start = .true. before
  calling  step  again.  this is the only situation in which  start
  should be altered.
  output from  step

successful step --

  the subroutine returns after each successful step with  start  and
  crash  set .false. .  x  represents the independent variable
  advanced one step of length  hold  from its value on input and  y
  the solution vector at the new value of  x .  all other parameters
  represent information corresponding to the new  x  needed to
  continue the integration.

unsuccessful step --

  when the error tolerance is too small for machine precision,
  the subroutine returns without taking a step and  crash = .true. .
  an appropriate step size and error tolerance for continuing are
  estimated and all other information is restored as upon input
  before returning.  to continue with the larger tolerance, the user
  just calls the code again.  a restart is neither required nor
  desirable.
*/
void shift::core::integrator::ODE113::step() {
  double temp2;
  double temp4;
  double temp5;
  double tau;
  double xold;
  double erk;
  double erkm1;
  double erkm2;
  double erkp1;
  double err;
  double hnew;
  double rho;

  unsigned int kp1;
  int kp2;
  int km1;
  int km2;
  int im1;
  int nsm2;
  unsigned int nsp2;
  unsigned int limit1;
  int ip1;
  int knew;

  /* check if step size or error tolerance is too small for machine
  precision. if first step, initialize phi array and estimate a
  starting step size.

  if step size is too small, determine an acceptable one */
  crash_ = true;
  h_ = fmax(h_, fouru_);
  double p5eps = 0.5 * eps_;

  /*  if error tolerance is too small, increase it to an acceptable value */
  double round = 0.0;
  for(size_t l = 0; l < neqn_; l++) {
    round += pow(yy_[l] / wt_[l], 2);
  }
  round = twou_ * sqrt(round);
  if(p5eps < round) {
    eps_ = 2.0 * round * (1.0 + fouru_);
    return;
  }
  crash_ = false;

  if(start_) {
    /* initialize: compute appropriate step size for first step */
    iconv_ = 1;
    StepFunction(x_, yy_);
    if(newton_raphson_failure_) {
      std::cerr << "Newton Raphson Failure" << std::endl;
      return;
    }
    double sum = 0.0;
    for(size_t l = 0; l < neqn_; l++) {
      phi_[1][l] = dy_[l];
      phi_[2][l] = 0.0;
      sum += pow(dy_[l] / wt_[l], 2);
    }
    sum = sqrt(sum);
    if(eps_ < 16.0 * sum * h_ * h_) {
      h_ = 0.25 * sqrt(eps_ / sum);
    }

    h_ = fmax(h_, fouru_ * x_);
    h_ = fmin(h_, hmax_);
    hold_ = 0.0;
    k_ = 1;
    kold_ = 0;
    start_ = false;
    phase1_ = true;
    nornd_ = true;
    if(p5eps <= 100.0 * round) {
      nornd_ = false;
      for(size_t l = 0; l < neqn_; l++) {
        phi_[15][l] = 0.0;
      }
    }
  }

  ifail_ = 0;

  while(true) {
    iconv_ = 0;

    /*  compute coefficients of formulas for this step; avoid computing
    those quantities not changed when step size is not changed.  */
    kp1 = k_ + 1;
    kp2 = k_ + 2;
    km1 = k_ - 1;
    km2 = k_ - 2;

    /*  ns is the number of steps taken with size h, including the current
       one; when k_ < ns_, no coefficients change */
    if(h_ != hold_) {
      ns_ = 0;
    }

    if(ns_ <= kold_) {
      ns_++;
    }

    unsigned int nsp1 = ns_ + 1;
    if(k_ >= ns_) {
      /* update components of alpha[], beta[], psi[], sig[] */
      beta_[ns_] = 1.0;
      alpha_[ns_] = 1.0 / ns_;
      double temp1 = h_ * ns_;
      sig_[nsp1] = 1.0;
      if(k_ >= nsp1) {
        for(unsigned int i = nsp1; i <= k_; i++) {
          im1 = i - 1;
          temp2 = psi_[im1];
          psi_[im1] = temp1;
          beta_[i] = beta_[im1] * psi_[im1] / temp2;
          temp1 = temp2 + h_;
          alpha_[i] = h_ / temp1;
          sig_[i + 1] = i * alpha_[i] * sig_[i];
        }
      }
      psi_[k_] = temp1;

      /*  compute coefficients g[]
          initialize v[] and set w[]. g[2] was initialized at creation */
      if(ns_ <= 1) {
        for(unsigned int iq = 1; iq <= k_; iq++) {
          v_[iq] = 1.0 / (iq * (iq + 1));
          w_[iq] = v_[iq];
        }
      } else {
        /*  if order was raised, update diagonal part of v[] */
        if(k_ > kold_) {
          v_[k_] = 1.0 / (k_ * kp1);
          nsm2 = ns_ - 2;
          if(nsm2 >= 1) {
            for(int j = 1; j <= nsm2; j++) {
              int i = k_ - j;
              v_[i] = v_[i] - alpha_[j + 1] * v_[i + 1];
            }
          }
        }
        /*  update v[] and set w([] */
        limit1 = kp1 - ns_;
        temp5 = alpha_[ns_];
        for(unsigned int iq = 1; iq <= limit1; iq++) {
          v_[iq] -= temp5 * v_[iq + 1];
          w_[iq] = v_[iq];
        }
        g_[nsp1] = w_[1];
      }
      /*  compute g[] in the work vector w[] */
      nsp2 = ns_ + 2;
      if(kp1 >= nsp2) {
        for(unsigned int i = nsp2; i <= kp1; i++) {
          for(unsigned int iq = 1; iq <= kp2 - i; iq++) {
            w_[iq] -= alpha_[i - 1] * w_[iq + 1];
          }
          g_[i] = w_[1];
        }
      }
    }

    /* predict a solution p[], evaluate derivatives using predicted
      solution, estimate local error at order k and errors at orders k,
      k-1, k-2 as if constant step size were used.
      change phi to phi star  */
    if(k_ >= nsp1) {
      for(unsigned int i = nsp1; i <= k_; i++) {
        for(unsigned int l = 0; l < neqn_; l++) {
          phi_[i][l] = beta_[i] * phi_[i][l];
        }
      }
    }

    /*  predict solution and differences */
    for(unsigned int l = 0; l < neqn_; l++) {
      phi_[kp2][l] = phi_[kp1][l];
      phi_[kp1][l] = 0.0;
      p_[l] = 0.0;
    }

    for(unsigned int j = 1; j <= k_; j++) {
      int i = kp1 - j;
      ip1 = i + 1;
      for(unsigned int l = 0; l < neqn_; l++) {
        p_[l] += g_[i] * phi_[i][l];
        phi_[i][l] += phi_[ip1][l];
      }
    }

    if(nornd_) {
      for(unsigned int l = 0; l < neqn_; l++) {
        p_[l] = yy_[l] + h_ * p_[l];
      }
    } else {
      for(unsigned int l = 0; l < neqn_; l++) {
        tau = h_ * p_[l] - phi_[15][l];
        p_[l] = yy_[l] + tau;
        phi_[16][l] = (p_[l] - yy_[l]) - tau;
      }
    }

    xold = x_;
    x_ += h_;
    StepFunction(x_, p_);
    if(newton_raphson_failure_) {
      std::cerr << "Error: Newton-Raphson failure" << std::endl;
      return;
    }

    /*  estimate errors at orders k, k-1, k-2 */
    erkm2 = 0.0;
    erkm1 = 0.0;
    erk = 0.0;
    if(km2 > 0) {
      for(unsigned int l = 0; l < neqn_; l++) {
        temp4 = dy_[l] - phi_[1][l];
        yerr_[l] = pow(temp4 / wt_[l], 2);
        erkm2 += pow((phi_[km1][l] + temp4) / wt_[l], 2);
        erkm1 += pow((phi_[k_][l] + temp4) / wt_[l], 2);
        erk += yerr_[l];
      }
      erkm2 = h_ * sig_[km1] * gstr_[km2] * sqrt(erkm2);
      erkm1 = h_ * sig_[k_] * gstr_[km1] * sqrt(erkm1);
    } else if(km1 > 0) {
      for(unsigned int l = 0; l < neqn_; l++) {
        temp4 = dy_[l] - phi_[1][l];
        yerr_[l] = pow(temp4 / wt_[l], 2);
        erkm1 += pow((phi_[k_][l] + temp4) / wt_[l], 2);
        erk += yerr_[l];
      }
      erkm1 = h_ * sig_[k_] * gstr_[km1] * sqrt(erkm1);
    } else {
      for(unsigned int l = 0; l < neqn_; l++) {
        temp4 = dy_[l] - phi_[1][l];
        yerr_[l] = pow(temp4 / wt_[l], 2);
        erk += yerr_[l];
      }
    }
    temp5 = h_ * sqrt(erk);
    err = temp5 * (g_[k_] - g_[kp1]);
    erk = temp5 * sig_[kp1] * gstr_[k_];
    knew = k_;

    /*  test if order should be lowered */
    if((km2 > 0) && (fmax(erkm1, erkm2) <= erk)) {
      knew = km1;
    } else if((km2 == 0) && (erkm1 <= 0.5 * erk)) {
      knew = km1;
    }

    if(err <= eps_) {
      break;
    }

    /* the step is unsuccessful. restore x, phi[][], psi[]. if third
       consecutive failure, set order to one. if step fails more than three
       times, consider an optimal step size. double error tolerance and
       return if estimated step size is too small for machine precision.

       restore x, phi[][] and psi[] */
    phase1_ = false;
    x_ = xold;
    iconv_ = -1;
    for(unsigned int i = 1; i <= k_; i++) {
      ip1 = i + 1;
      for(unsigned int l = 0; l < neqn_; l++) {
        phi_[i][l] = (phi_[i][l] - phi_[ip1][l]) / beta_[i];
      }
    }

    if(k_ >= 2) {
      for(unsigned int i = 2; i <= k_; i++) {
        psi_[i - 1] = psi_[i] - h_;
      }
    }

    /* on third failure, set order to one. thereafter, use optimal step size */
    ifail_++;
    temp2 = 0.5;
    if((ifail_ > 3) && (p5eps < 0.25 * erk)) {
      temp2 = sqrt(p5eps / erk);
    }

    if(ifail_ >= 3) {
      knew = 1;
    }

    k_ = knew;
    h_ = temp2 * h_;
    if(h_ < fouru_ * x_) {
      crash_ = true;
      h_ = fouru_ * x_;
      eps_ += eps_;
      return;
    }
  }

  /* the step is successful. correct the predicted solution, evaluate
  the derivatives using the corrected solution and update the
  differences. determine best order and step size for next step. */
  kold_ = k_;
  hold_ = h_;

  /*  correct and evaluate */
  double temp1 = h_ * g_[kp1];
  if(nornd_) {
    for(unsigned int l = 0; l < neqn_; l++) {
      yy_[l] = p_[l] + temp1 * (dy_[l] - phi_[1][l]);
    }
  } else {
    for(unsigned int l = 0; l < neqn_; l++) {
      rho = temp1 * (dy_[l] - phi_[1][l]) - phi_[16][l];
      yy_[l] = p_[l] + rho;
      phi_[15][l] = (yy_[l] - p_[l]) - rho;
    }
  }

  iconv_ = 1;
  StepFunction(x_, yy_);
  if(newton_raphson_failure_) {
    std::cerr << "Newton Raphson failure" << std::endl;
    return;
  }

  /*  update differences for next step */
  for(unsigned int l = 0; l < neqn_; l++) {
    phi_[kp1][l] = dy_[l] - phi_[1][l];
    phi_[kp2][l] = phi_[kp1][l] - phi_[kp2][l];
  }
  for(unsigned int i = 1; i <= k_; i++) {
    for(unsigned int l = 0; l < neqn_; l++) {
      phi_[i][l] += phi_[kp1][l];
    }
  }

  /*  estimate error at order k + 1 unless:
      in first phase when always raise order,
      already decided to lower order,
      step size not constant so estimate unreliable */
  erkp1 = 0.0;
  if((knew == km1) || (k_ == 12)) {
    phase1_ = false;
  }

  if(phase1_) {
    /* raise order */
    k_ = kp1;
    erk = erkp1;
  } else if(knew == km1) {
    /* lower order */
    k_ = km1;
    erk = erkm1;
  } else if(kp1 <= ns_) {
    for(unsigned int l = 0; l < neqn_; l++) {
      erkp1 += pow(phi_[kp2][l] / wt_[l], 2);
    }
    erkp1 = h_ * gstr_[kp1] * sqrt(erkp1);

    /* using estimated error at order k+1, determine appropriate order
       for next step */
    if(k_ > 1) {
      if(erkm1 <= (fmin(erk, erkp1))) {
        /* lower order */
        k_ = km1;
        erk = erkm1;
      } else if((erkp1 < erk) && (k_ < 12)) {
        /*  here erkp1 < erk < max(erkm1,erkm2) else order would have
            been lowered in block 2. thus order is to be raised */
        /* raise order */
        k_ = kp1;
        erk = erkp1;
      }
    } else if(erkp1 < 0.5 * erk) {
      /* raise order */
      k_ = kp1;
      erk = erkp1;
    }
  }

  hnew = h_ + h_;
  if((!phase1_) && (p5eps < erk * two_[k_ + 1])) {
    hnew = h_;
    if(p5eps < erk) {
      hnew = fmin(0.9, pow(p5eps / erk, 1.0 / (k_ + 1)));
      hnew = fmax(hnew, 0.5);
      hnew = h_ * hnew;
      hnew = fmax(hnew, fouru_);
    }
  }
  h_ = hnew;
  return;
}

void shift::core::integrator::ODE113::get_current_time(double& time, double& delta_time) {
  time = x_;
  delta_time = x_ - t_; // @TODO verify this is correct
}
