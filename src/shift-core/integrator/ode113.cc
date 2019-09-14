#include "shift-core/integrator/ode113.h"
#include "shift-core/context.h"
#include "shift-core/factory.h"

#include <math.h>

const double Shift::Integrator::ODE113::twou_ =
  std::numeric_limits<double>::epsilon() * 2.0;

const double Shift::Integrator::ODE113::fouru_ =
  std::numeric_limits<double>::epsilon() * 4.0;

const double Shift::Integrator::ODE113::gstr_[14] = {
  0.0,    0.5,     0.0833,  0.0417,  0.0264,  0.0188,  0.0143,
  0.0114, 0.00936, 0.00789, 0.00679, 0.00592, 0.00524, 0.00468};

const double Shift::Integrator::ODE113::two_[14] = {
  0.0,   2.0,   4.0,   8.0,    16.0,   32.0,   64.0,
  128.0, 256.0, 512.0, 1024.0, 2048.0, 4096.0, 8192.0};

const double Shift::Integrator::ODE113::stability_limits_[17] = {
  1.0,  2.0,  2.26, 1.86, 1.4, 1.03, 0.78, 0.58, 0.44,
  0.34, 0.26, 0.21, 0.6,  0.2, 0.2,  0.2,  0.2};

bool Shift::Integrator::ODE113::registered_ =
  Shift::Factory<Shift::Integrator::Base>::get()->registerModule(
    "ode113", Shift::Integrator::ODE113::create);

std::unique_ptr<Shift::Integrator::ODE113> Shift::Integrator::ODE113::create() {
  return std::make_unique<Shift::Integrator::ODE113>();
}

void Shift::Integrator::ODE113::addParameters() {
  addParameter("safety_factor", &safety_factor_,
               "adjustable safety factor for numerical integration: defaults "
               "to 0.0005 if not provided",
               false);
  addParameter("hmax", &hmax_,
               "largest internal time step allowed by numerical integrator, "
               "defaults to 0.01 if not provided.",
               false);
  addParameter("hmax", &hmax_,
               "smallest internal time step allowed by numerical integrator, "
               "defaults to 0.000001 if not provided.",
               false);
  addParameter("omega_n_max", &omega_n_max_,
               "largest value for omega_n used in Baumgarte constraint "
               "stabilization, defaults to 1000.0 if not provided",
               false);
  addParameter("omega_n_max", &omega_n_max_,
               "largest value for omega_n used in Baumgarte constraint "
               "stabilization, defaults to 1000.0 if not provided",
               false);
  addParameter("integral_gain", &integral_gain_,
               "integral gain used in baumgarte constraint stabilization, "
               "defaults to 1000.0 if not provided",
               false);
  addParameter("time_span", &time_span_,
               "time span for numerical integration, defaults to 10 seconds if "
               "not provided ",
               false);
  addParameter("time_step", &time_step_,
               "time step used for numerical integration, defaults to 0.01 "
               "seconds if not provided.",
               false);
}

void Shift::Integrator::ODE113::connect() {}

void Shift::Integrator::ODE113::initialize() {

  integral_error_.resize(context_->chordConstraintCount() * 6);
  std::fill(integral_error_.begin(), integral_error_.end(), 0);

  step_count_ = ceil(time_span_ / time_step_) + 1;

#if 0
  yy_.resize(neqn_);

    numericalIntegrator->ydata = calloc(
      numericalIntegrator->neqn * numericalIntegrator->stepCount, sizeof(double));
    numericalIntegrator->tdata =
      calloc(numericalIntegrator->stepCount, sizeof(double));
    memcpy(numericalIntegrator->ydata, numericalIntegrator->y,
           numericalIntegrator->neqn * sizeof(double));

    numericalIntegrator->dy = calloc(numericalIntegrator->neqn, sizeof(double));

    s2_context_connectForceModules(context);

    numericalIntegrator->yy = calloc(numericalIntegrator->neqn, sizeof(double));
    memcpy(numericalIntegrator->yy, numericalIntegrator->y,
           numericalIntegrator->neqn * sizeof(double));

    numericalIntegrator->dyout =
      calloc(numericalIntegrator->neqn, sizeof(double));

    numericalIntegrator->wt = calloc(numericalIntegrator->neqn, sizeof(double));

    numericalIntegrator->p = calloc(numericalIntegrator->neqn, sizeof(double));

    numericalIntegrator->yerr = calloc(numericalIntegrator->neqn, sizeof(double));

    numericalIntegrator->phi = s2_array_malloc2D(17, numericalIntegrator->neqn);

#endif
}
void Shift::Integrator::ODE113::integrate() {

#if 0
  context_->solve();


    if(iconv_ == 1) {
      h = fmax(x_ - t_, hmin_); /* limit step size */

      double omega_n = stability_limits_[k_] / h * safety_factor_;

      omega_n = fmin(omega_n, omega_n_max_); /* limit omega_n */

      omega_n =
        omega_n_prev + smoothing_filter_ * (omega_n - omega_n_prev);

      omega_n_prev_ = omega_n;
      baumgarte_2_omega_n_ = 2.0 * omega_n;
      baumgarte_omega_n_2_ = omega_n * omega_n;

      double rho_dt = h * integral_gain_;
      for(int i = 0; i < context->chordConstraintCount * 6; i++) {
        integral_error_[i] += context->cs[i] * rho_dt;
      }

      t_ = x_;
      if(x_ >= tdata_[step] + context->modelParameters.timeStep) {
        context_->writeStates(++step_);
        iteration_count_ = 0;
      }
    }
    iconv_ = 0;
    return;
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

    when the error tolerance is too small for the machine precision,
    the subroutine returns without taking a step and  crash = .true. .
    an appropriate step size and error tolerance for continuing are
    estimated and all other information is restored as upon input
    before returning.  to continue with the larger tolerance, the user
    just calls the code again.  a restart is neither required nor
    desirable.
  */
  void s2_ode113b_step(void f(double y[], s2_context_t* context),
                       s2_context_t* context) {

    s2_numericalintegrator_ode113b_t* numericalIntegrator =
      (s2_numericalintegrator_ode113b_t*)context->numericalIntegrator;

    double p5eps;
    double round;
    double sum;
    double temp1;
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

    int kp1;
    int kp2;
    int km1;
    int km2;
    int nsp1;
    int im1;
    int iq;
    int nsm2;
    int nsp2;
    int limit1;
    int ip1;
    int knew;

    /*
    begin block 0
    check if step size or error tolerance is too small for machine
    precision. if first step, initialize phi array and estimate a
    starting step size.

    if step size is too small, determine an acceptable one */
    numericalIntegrator->crash = true;
    numericalIntegrator->h =
      fmax(numericalIntegrator->h, numericalIntegrator->fouru);
    p5eps = 0.5 * numericalIntegrator->eps;

    /*  if error tolerance is too small, increase it to an acceptable value */
    round = 0.0;
    for(int l = 0; l < numericalIntegrator->neqn; l++) {
      round += pow(numericalIntegrator->yy[l] / numericalIntegrator->wt[l], 2);
    }
    round = numericalIntegrator->twou * sqrt(round);
    if(p5eps < round) {
      numericalIntegrator->eps =
        2.0 * round * (1.0 + numericalIntegrator->fouru);
      return;
    }
    numericalIntegrator->crash = false;

    if(numericalIntegrator->start) {

      /* initialize. compute appropriate step size for first step */
      numericalIntegrator->iconv = 1;
      f(numericalIntegrator->yy, context);
      if(numericalIntegrator->newtonRaphsonFailure) {
        return;
      }
      sum = 0.0;
      for(int l = 0; l < numericalIntegrator->neqn; l++) {
        numericalIntegrator->phi[1][l] = numericalIntegrator->dy[l];
        numericalIntegrator->phi[2][l] = 0.0;
        sum += pow(numericalIntegrator->dy[l] / numericalIntegrator->wt[l], 2);
      }
      sum = sqrt(sum);
      if(numericalIntegrator->eps <
         16.0 * sum * numericalIntegrator->h * numericalIntegrator->h) {
        numericalIntegrator->h = 0.25 * sqrt(numericalIntegrator->eps / sum);
      }

      numericalIntegrator->h =
        fmax(numericalIntegrator->h,
             numericalIntegrator->fouru * numericalIntegrator->x);
      numericalIntegrator->h =
        fmin(numericalIntegrator->h, numericalIntegrator->hmax);
      numericalIntegrator->hold = 0.0;
      numericalIntegrator->k = 1;
      numericalIntegrator->kold = 0;
      numericalIntegrator->start = false;
      numericalIntegrator->phase1 = true;
      numericalIntegrator->nornd = true;
      if(p5eps <= 100.0 * round) {
        numericalIntegrator->nornd = false;
        for(int l = 0; l < numericalIntegrator->neqn; l++) {
          numericalIntegrator->phi[15][l] = 0.0;
        }
      }
    }

    numericalIntegrator->ifail = 0;

    while(true) {
      numericalIntegrator->iconv = 0;

      /*    begin block 1
      compute coefficients of formulas for this step. avoid computing
      those quantities not changed when step size is not changed.  */

      kp1 = numericalIntegrator->k + 1;
      kp2 = numericalIntegrator->k + 2;
      km1 = numericalIntegrator->k - 1;
      km2 = numericalIntegrator->k - 2;

      /*  ns is the number of steps taken with size h, including the current
         one. when k.lt.ns, no coefficients change */

      if(numericalIntegrator->h != numericalIntegrator->hold) {
        numericalIntegrator->ns = 0;
      }

      if(numericalIntegrator->ns <= numericalIntegrator->kold) {
        numericalIntegrator->ns++;
      }

      nsp1 = numericalIntegrator->ns + 1;

      if(numericalIntegrator->k >= numericalIntegrator->ns) {

        /*  compute those components of alpha(*),beta(*),psi(*),sig(*) which
            are changed */
        numericalIntegrator->beta[numericalIntegrator->ns] = 1.0;
        numericalIntegrator->alpha[numericalIntegrator->ns] =
          1.0 / numericalIntegrator->ns;
        temp1 = numericalIntegrator->h * numericalIntegrator->ns;
        numericalIntegrator->sig[nsp1] = 1.0;
        if(numericalIntegrator->k >= nsp1) {
          for(int i = nsp1; i <= numericalIntegrator->k; i++) {
            im1 = i - 1;
            temp2 = numericalIntegrator->psi[im1];
            numericalIntegrator->psi[im1] = temp1;
            numericalIntegrator->beta[i] = numericalIntegrator->beta[im1] *
                                           numericalIntegrator->psi[im1] /
                                           temp2;
            temp1 = temp2 + numericalIntegrator->h;
            numericalIntegrator->alpha[i] = numericalIntegrator->h / temp1;
            numericalIntegrator->sig[i + 1] =
              i * numericalIntegrator->alpha[i] * numericalIntegrator->sig[i];
          }
        }
        numericalIntegrator->psi[numericalIntegrator->k] = temp1;

        /*  compute coefficients g(*)
            initialize v(*) and set w(*). g(2) was initialized at creation */
        if(numericalIntegrator->ns <= 1) {
          for(int iq = 1; iq <= numericalIntegrator->k; iq++) {
            numericalIntegrator->v[iq] = 1.0 / (iq * (iq + 1));
            numericalIntegrator->w[iq] = numericalIntegrator->v[iq];
          }
        } else {
          /*  if order was raised, update diagonal part of v(*) */
          if(numericalIntegrator->k > numericalIntegrator->kold) {
            numericalIntegrator->v[numericalIntegrator->k] =
              1.0 / (numericalIntegrator->k * kp1);
            nsm2 = numericalIntegrator->ns - 2;
            if(nsm2 >= 1) {
              for(int j = 1; j <= nsm2; j++) {
                int i = numericalIntegrator->k - j;
                numericalIntegrator->v[i] = numericalIntegrator->v[i] -
                                            numericalIntegrator->alpha[j + 1] *
                                              numericalIntegrator->v[i + 1];
              }
            }
          }
          /*  update v(*) and set w(*) */
          limit1 = kp1 - numericalIntegrator->ns;
          temp5 = numericalIntegrator->alpha[numericalIntegrator->ns];
          for(iq = 1; iq <= limit1; iq++) {
            numericalIntegrator->v[iq] -=
              temp5 * numericalIntegrator->v[iq + 1];
            numericalIntegrator->w[iq] = numericalIntegrator->v[iq];
          }
          numericalIntegrator->g[nsp1] = numericalIntegrator->w[1];
        }
        /*  compute g(*) in the work vector w(*) */
        nsp2 = numericalIntegrator->ns + 2;
        if(kp1 >= nsp2) {
          for(int i = nsp2; i <= kp1; i++) {
            for(iq = 1; iq <= kp2 - i; iq++) {
              numericalIntegrator->w[iq] -= numericalIntegrator->alpha[i - 1] *
                                            numericalIntegrator->w[iq + 1];
            }
            numericalIntegrator->g[i] = numericalIntegrator->w[1];
          }
        }
      }

      /* end block 1

        begin block 2
        predict a solution p(*), evaluate derivatives using predicted
        solution, estimate local error at order k and errors at orders k,
        k-1, k-2 as if constant step size were used.

        change phi to phi star  */

      if(numericalIntegrator->k >= nsp1) {
        for(int i = nsp1; i <= numericalIntegrator->k; i++) {
          for(int l = 0; l < numericalIntegrator->neqn; l++) {
            numericalIntegrator->phi[i][l] =
              numericalIntegrator->beta[i] * numericalIntegrator->phi[i][l];
          }
        }
      }

      /*  predict solution and differences */
      for(int l = 0; l < numericalIntegrator->neqn; l++) {
        numericalIntegrator->phi[kp2][l] = numericalIntegrator->phi[kp1][l];
        numericalIntegrator->phi[kp1][l] = 0.0;
        numericalIntegrator->p[l] = 0.0;
      }
      for(int j = 1; j <= numericalIntegrator->k; j++) {
        int i = kp1 - j;
        ip1 = i + 1;
        for(int l = 0; l < numericalIntegrator->neqn; l++) {
          numericalIntegrator->p[l] +=
            numericalIntegrator->g[i] * numericalIntegrator->phi[i][l];
          numericalIntegrator->phi[i][l] += numericalIntegrator->phi[ip1][l];
        }
      }
      if(numericalIntegrator->nornd) {
        for(int l = 0; l < numericalIntegrator->neqn; l++)
          numericalIntegrator->p[l] =
            numericalIntegrator->yy[l] +
            numericalIntegrator->h * numericalIntegrator->p[l];
      } else {
        for(int l = 0; l < numericalIntegrator->neqn; l++) {
          tau = numericalIntegrator->h * numericalIntegrator->p[l] -
                numericalIntegrator->phi[15][l];
          numericalIntegrator->p[l] = numericalIntegrator->yy[l] + tau;
          numericalIntegrator->phi[16][l] =
            (numericalIntegrator->p[l] - numericalIntegrator->yy[l]) - tau;
        }
      }
      xold = numericalIntegrator->x;
      numericalIntegrator->x = numericalIntegrator->x + numericalIntegrator->h;
      f(numericalIntegrator->p, context);
      if(numericalIntegrator->newtonRaphsonFailure) {
        return;
      }

      /*  estimate errors at orders k, k-1, k-2 */
      erkm2 = 0.0;
      erkm1 = 0.0;
      erk = 0.0;
      if(km2 > 0) {
        for(int l = 0; l < numericalIntegrator->neqn; l++) {
          temp4 = numericalIntegrator->dy[l] - numericalIntegrator->phi[1][l];
          numericalIntegrator->yerr[l] =
            pow(temp4 / numericalIntegrator->wt[l], 2);
          erkm2 += pow((numericalIntegrator->phi[km1][l] + temp4) /
                         numericalIntegrator->wt[l],
                       2);
          erkm1 +=
            pow((numericalIntegrator->phi[numericalIntegrator->k][l] + temp4) /
                  numericalIntegrator->wt[l],
                2);
          erk += numericalIntegrator->yerr[l];
        }
        erkm2 = numericalIntegrator->h * numericalIntegrator->sig[km1] *
                numericalIntegrator->gstr[km2] * sqrt(erkm2);
        erkm1 = numericalIntegrator->h *
                numericalIntegrator->sig[numericalIntegrator->k] *
                numericalIntegrator->gstr[km1] * sqrt(erkm1);
      } else if(km1 > 0) {
        for(int l = 0; l < numericalIntegrator->neqn; l++) {
          temp4 = numericalIntegrator->dy[l] - numericalIntegrator->phi[1][l];
          numericalIntegrator->yerr[l] =
            pow(temp4 / numericalIntegrator->wt[l], 2);
          erkm1 +=
            pow((numericalIntegrator->phi[numericalIntegrator->k][l] + temp4) /
                  numericalIntegrator->wt[l],
                2);
          erk += numericalIntegrator->yerr[l];
        }
        erkm1 = numericalIntegrator->h *
                numericalIntegrator->sig[numericalIntegrator->k] *
                numericalIntegrator->gstr[km1] * sqrt(erkm1);
      } else {
        for(int l = 0; l < numericalIntegrator->neqn; l++) {
          temp4 = numericalIntegrator->dy[l] - numericalIntegrator->phi[1][l];
          numericalIntegrator->yerr[l] =
            pow(temp4 / numericalIntegrator->wt[l], 2);
          erk += numericalIntegrator->yerr[l];
        }
      }
      temp5 = numericalIntegrator->h * sqrt(erk);
      err = temp5 * (numericalIntegrator->g[numericalIntegrator->k] -
                     numericalIntegrator->g[kp1]);
      erk = temp5 * numericalIntegrator->sig[kp1] *
            numericalIntegrator->gstr[numericalIntegrator->k];
      knew = numericalIntegrator->k;

      /*  test if order should be lowered */
      if((km2 > 0) && (fmax(erkm1, erkm2) <= erk)) {
        knew = km1;
      } else if((km2 == 0) && (erkm1 <= 0.5 * erk)) {
        knew = km1;
      }

      if(err <= numericalIntegrator->eps) {
        break;
      }

      /*  begin block 3
         the step is unsuccessful. restore  x, phi(*,*), psi(*). if third
         consecutive failure, set order to one. if step fails more than three
         times, consider an optimal step size. double error tolerance and
         return if estimated step size is too small for machine precision.

         restore x, phi(*,*) and psi(*) */
      numericalIntegrator->phase1 = false;
      numericalIntegrator->x = xold;
      numericalIntegrator->iconv = -1;
      for(int i = 1; i <= numericalIntegrator->k; i++) {
        ip1 = i + 1;
        for(int l = 0; l < numericalIntegrator->neqn; l++) {
          numericalIntegrator->phi[i][l] = (numericalIntegrator->phi[i][l] -
                                            numericalIntegrator->phi[ip1][l]) /
                                           numericalIntegrator->beta[i];
        }
      }

      if(numericalIntegrator->k >= 2) {
        for(int i = 2; i <= numericalIntegrator->k; i++) {
          numericalIntegrator->psi[i - 1] =
            numericalIntegrator->psi[i] - numericalIntegrator->h;
        }
      }

      /* on third failure, set order to one. thereafter, use optimal step size
       */
      numericalIntegrator->ifail++;
      temp2 = 0.5;
      if((numericalIntegrator->ifail > 3) && (p5eps < 0.25 * erk)) {
        temp2 = sqrt(p5eps / erk);
      }

      if(numericalIntegrator->ifail >= 3) {
        knew = 1;
      }

      numericalIntegrator->k = knew;
      numericalIntegrator->h = temp2 * numericalIntegrator->h;
      if(numericalIntegrator->h <
         numericalIntegrator->fouru * numericalIntegrator->x) {
        numericalIntegrator->crash = true;
        numericalIntegrator->h =
          numericalIntegrator->fouru * numericalIntegrator->x;
        numericalIntegrator->eps += numericalIntegrator->eps;
        return;
      }
    }

    /*
      begin block 4

      the step is successful. correct the predicted solution, evaluate
      the derivatives using the corrected solution and update the
      differences. determine best order and step size for next step. */

    numericalIntegrator->kold = numericalIntegrator->k;
    numericalIntegrator->hold = numericalIntegrator->h;

    /*  correct and evaluate */
    temp1 = numericalIntegrator->h * numericalIntegrator->g[kp1];
    if(numericalIntegrator->nornd) {
      for(int l = 0; l < numericalIntegrator->neqn; l++)
        numericalIntegrator->yy[l] =
          numericalIntegrator->p[l] +
          temp1 * (numericalIntegrator->dy[l] - numericalIntegrator->phi[1][l]);
    } else {
      for(int l = 0; l < numericalIntegrator->neqn; l++) {
        rho = temp1 *
                (numericalIntegrator->dy[l] - numericalIntegrator->phi[1][l]) -
              numericalIntegrator->phi[16][l];
        numericalIntegrator->yy[l] = numericalIntegrator->p[l] + rho;
        numericalIntegrator->phi[15][l] =
          (numericalIntegrator->yy[l] - numericalIntegrator->p[l]) - rho;
      }
    }

    numericalIntegrator->iconv = 1;
    f(numericalIntegrator->yy, context);
    if(numericalIntegrator->newtonRaphsonFailure) {
      return;
    }

    /*  update differences for next step */
    for(int l = 0; l < numericalIntegrator->neqn; l++) {
      numericalIntegrator->phi[kp1][l] =
        numericalIntegrator->dy[l] - numericalIntegrator->phi[1][l];
      numericalIntegrator->phi[kp2][l] =
        numericalIntegrator->phi[kp1][l] - numericalIntegrator->phi[kp2][l];
    }
    for(int i = 1; i <= numericalIntegrator->k; i++) {
      for(int l = 0; l < numericalIntegrator->neqn; l++) {
        numericalIntegrator->phi[i][l] += numericalIntegrator->phi[kp1][l];
      }
    }

    /*  estimate error at order k + 1 unless:
        in first phase when always raise order,
        already decided to lower order,
        step size not constant so estimate unreliable */
    erkp1 = 0.0;
    if((knew == km1) || (numericalIntegrator->k == 12)) {
      numericalIntegrator->phase1 = false;
    }

    if(numericalIntegrator->phase1) {
      /* raise order */
      numericalIntegrator->k = kp1;
      erk = erkp1;
    } else if(knew == km1) {
      /* lower order */
      numericalIntegrator->k = km1;
      erk = erkm1;
    } else if(kp1 <= numericalIntegrator->ns) {
      for(int l = 0; l < numericalIntegrator->neqn; l++) {
        erkp1 +=
          pow(numericalIntegrator->phi[kp2][l] / numericalIntegrator->wt[l], 2);
      }
      erkp1 =
        numericalIntegrator->h * numericalIntegrator->gstr[kp1] * sqrt(erkp1);

      /* using estimated error at order k+1, determine appropriate order
         for next step */
      if(numericalIntegrator->k > 1) {
        if(erkm1 <= (fmin(erk, erkp1))) {
          /* lower order */
          numericalIntegrator->k = km1;
          erk = erkm1;
        } else if((erkp1 < erk) && (numericalIntegrator->k < 12)) {
          /*  here erkp1 < erk < max(erkm1,erkm2) else order would have
              been lowered in block 2. thus order is to be raised */
          /* raise order */
          numericalIntegrator->k = kp1;
          erk = erkp1;
        }
      } else if(erkp1 < 0.5 * erk) {
        /* raise order */
        numericalIntegrator->k = kp1;
        erk = erkp1;
      }
    }

    hnew = numericalIntegrator->h + numericalIntegrator->h;
    if((!numericalIntegrator->phase1) &&
       (p5eps < erk * numericalIntegrator->two[numericalIntegrator->k + 1])) {

      hnew = numericalIntegrator->h;
      if(p5eps < erk) {
        hnew = fmin(0.9, pow(p5eps / erk, 1.0 / (numericalIntegrator->k + 1)));
        hnew = fmax(hnew, 0.5);
        hnew = numericalIntegrator->h * hnew;
        hnew = fmax(hnew, numericalIntegrator->fouru);
      }
    }
    numericalIntegrator->h = hnew;
    return;
  }

  /*!
  @brief subroutine de integrates a system of neqn first order ordinary
    differential equations of the form

             dy(i)/dt = f(t,y(1),y(2),...,y(neqn))
             y(i) given at  t .

    the subroutine integrates from  t  to  tout .  on return the
    parameters in the call list are initialized for continuing the
    integration.  the user has only to define a new value  tout
    and call  de  again.

    de  calls two codes, the integrator  step  and the interpolation
    routine  intrp .  step  uses a modified divided difference form of
    the adams pece formulas and local extrapolation.  it adjusts the
    order and step size to control the local error.  normally each call
    to  step  advances the solution one step in the direction of  tout .
    though never beyond t + 10*(tout-t), and calls  intrp  to
    interpolate the solution at  tout .  an option is provided to stop
    the integration at  tout  but it should be used only if it is
    impossible to continue the integration beyond  tout .

    this code is completely explained and documented in the text,
    computer solution of ordinary differential equations:  the initial
    value problem  by l. f. shampine and m. k. gordon.

    the parameters for  de  are:
      f -- subroutine f(t,y,yp) to evaluate derivatives yp(i)=dy(i)/dt
      neqn -- number of equations to be integrated
      y(*) -- solution vector at  t
      t -- independent variable
      tout -- point at which solution is desired
      relerr,abserr -- relative and absolute error tolerances for local
           error test.  at each step the code requires
             abs(local error) .le. abs(y)*relerr + abserr
           for each component of the local error and solution vectors
      iflag -- indicates status of integration

    first call to de --

    the user must provide storage in his calling program for the array
    in the call list, y(neqn), declare  f  in an external statement,
    supply the subroutine f(t,y,yp) to evaluate
      dy(i)/dt = yp(i) = f(t,y(1),y(2),...,y(neqn))
    and initialize the parameters:
      neqn -- number of equations to be integrated
      y(*) -- vector of initial conditions
      t -- starting point of integration
      tout -- point at which solution is desired
      relerr,abserr -- relative and absolute local error tolerances
      iflag -- +1,-1.  indicator to initialize the code.  normal input
           is +1.  the user should set iflag=-1 only if it is
           impossible to continue the integration beyond  tout .
    all parameters except  f ,  neqn  and  tout  may be altered by the
    code on output so must be variables in the calling program.

    output from  de --

      neqn -- unchanged
      y(*) -- solution at  t
      t -- last point reached in integration.  normal return has
           t = tout .
      tout -- unchanged
      relerr,abserr -- normal return has tolerances unchanged.  iflag=3
           signals tolerances increased
      iflag = 2 -- normal return.  integration reached  tout
            = 3 -- integration did not reach  tout  because error
                   tolerances too small.  relerr ,  abserr  increased
                   appropriately for continuing
            = 5 -- integration did not reach  tout  because equations
                   appear to be stiff
            = 6 -- invalid input parameters (fatal error)
           the value of  iflag  is returned negative when the input
           value is negative and the integration does not reach  tout ,
           i.e., -3, -4, -5.

    subsequent calls to  de --

    subroutine  de  returns with all information needed to continue
    the integration.  if the integration reached  tout , the user need
    only define a new  tout  and call again.  if the integration did not
    reach  tout  and the user wants to continue, he just calls again.
    the output value of  iflag  is the appropriate input value for
    subsequent calls.  the only situation in which it should be altered
    is to stop the integration internally at the new  tout , i.e.,
    change output  iflag=2  to input  iflag=-2 .  error tolerances may
    be changed by the user before continuing.  all other parameters must
    remain unchanged.
  */
  void s2_ode113b_de(void f(double y[], s2_context_t* context),
                     s2_context_t* context) {

    s2_numericalintegrator_ode113b_t* numericalIntegrator =
      (s2_numericalintegrator_ode113b_t*)context->numericalIntegrator;

    double releps;
    double abseps;

    numericalIntegrator->step = 0;
    numericalIntegrator->eps = fmax(context->modelParameters.relativeError,
                                    context->modelParameters.absoluteError);

    /* test for improper parameters */
    if(numericalIntegrator->neqn < 1 ||
       context->modelParameters.timeSpan == 0.0 ||
       context->modelParameters.relativeError < 0.0 ||
       context->modelParameters.absoluteError < 0.0 ||
       numericalIntegrator->iflag < 1 || numericalIntegrator->iflag > 5 ||
       numericalIntegrator->eps < numericalIntegrator->twou) {

      printf(
        "neqn=%d, time=%g, timeEnd=%g, relerr=%g, abserr=%g, iflag=%d, eps=%g, "
        "twou=%g\n",
        numericalIntegrator->neqn, 0.0, context->modelParameters.timeSpan,
        context->modelParameters.relativeError,
        context->modelParameters.absoluteError, numericalIntegrator->iflag,
        numericalIntegrator->eps, numericalIntegrator->twou);

      numericalIntegrator->iflag = 6;
      return;
    }

    /*  on each call, set interval of integration and counter for number of
        steps.  adjust input error tolerances to define weight vector.   */
    numericalIntegrator->abserr = context->modelParameters.absoluteError;
    numericalIntegrator->relerr = context->modelParameters.relativeError;

    releps = numericalIntegrator->relerr / numericalIntegrator->eps;
    abseps = numericalIntegrator->abserr / numericalIntegrator->eps;

    /*  on start and restart also set work variables x and yy(*), store the
        direction of integration and initialize the step size               */
    if(numericalIntegrator->iflag == 1) {
      numericalIntegrator->start = true;
      numericalIntegrator->k = 0;
      numericalIntegrator->kold = 0;
      numericalIntegrator->x = 0.0;
      memcpy(numericalIntegrator->yy, numericalIntegrator->y,
             numericalIntegrator->neqn * sizeof(double));
      numericalIntegrator->h = numericalIntegrator->hmax;

      for(int i = 0; i < 13; i++) {
        numericalIntegrator->psi[i] = 0;
      }
    }

    while(true) {
      /*  limit step size, set weight vector and take a step */
      numericalIntegrator->h =
        fmin(numericalIntegrator->h, numericalIntegrator->hmax);
      numericalIntegrator->h =
        fmin(numericalIntegrator->h,
             context->modelParameters.timeSpan - numericalIntegrator->x);
      for(int l = 0; l < numericalIntegrator->neqn; l++) {
        numericalIntegrator->wt[l] =
          releps * fabs(numericalIntegrator->yy[l]) + abseps;
      }

      s2_ode113b_step(f, context);

      if(numericalIntegrator->newtonRaphsonFailure) {
        fprintf(stderr, "NR Failure\n");
        return;
      }

      if(numericalIntegrator->x >= context->modelParameters.timeSpan) {
        numericalIntegrator->iflag = 2;
        return;
      }

      /*  If crash, increase error tolerances and return to
        calling program. Can call de again to continue integration with larger
        tolerances.*/
      if(numericalIntegrator->crash) {
        numericalIntegrator->iflag = 3;
        numericalIntegrator->relerr = numericalIntegrator->eps * releps;
        numericalIntegrator->abserr = numericalIntegrator->eps * abseps;
        memcpy(numericalIntegrator->y, numericalIntegrator->yy,
               numericalIntegrator->neqn * sizeof(double));
        numericalIntegrator->x = numericalIntegrator->t;
        return;
      } /*End if crash*/
    }   /*End While*/
  }

  /*!
  @brief placeholder for integrating using ode113b method
  */
  void s2_context_integrateEquationsOde113b(s2_context_t * context) {

    s2_ode113b_de(s2_ode113b_fcn, context);
    s2_numericalintegrator_ode113b_t* numericalIntegrator =
      (s2_numericalintegrator_ode113b_t*)context->numericalIntegrator;

    /*! if number of steps is less than total amount of space allocated, pad the
    states out to the end of the space allocated to avoid incorrect display in
    graphics

    @TODO need to add interpolation in graphics for data at irregular time
    interval reporting */
    if(numericalIntegrator->step < numericalIntegrator->stepCount - 1) {
      for(int i = numericalIntegrator->step; i < numericalIntegrator->stepCount;
          i++) {

        memcpy(numericalIntegrator->ydata + numericalIntegrator->neqn * i,
               numericalIntegrator->ydata +
                 numericalIntegrator->neqn * (numericalIntegrator->step - 1),
               numericalIntegrator->neqn * sizeof(double));
      }
    }

    if(numericalIntegrator->iflag == 3) {
      fprintf(stderr, "ODE113B was unable to continue.\n");
    }

    if(numericalIntegrator->newtonRaphsonFailure) {
      fprintf(stderr, "Newton Raphson failed, solver stopped\n");
    }
  }
}
#endif
}
