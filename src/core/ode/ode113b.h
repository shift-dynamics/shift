#ifndef _SHIFT_ODE113B_H_
#define _SHIFT_ODE113B_H_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

typedef struct s2_stack_t s2_stack_t;
typedef struct s2_context_t s2_context_t;

#include <stdbool.h>

/*!
@brief Attributes associated with numerical integrator
*/
typedef struct s2_numericalintegrator_ode113b_t {
  double* y;     /*!< y is a de copy of the states communicated to the
                    calling program. Calling program provides y as the
                    the initial conditions. de copies y into yy and
                    sends this to step. If step fails to advance solution,
                    de copies latest yy from step back to y and returns
                    this result to calling program for continuation. */
  double* dy;    /*!< This is the array de passes to step to use for
                    predicted and corrected state derivatives. If successful,
                    step computes dy and returns it to de as dy. If step
                    is unsuccessful, de just ignores dy. */
  double* ydata; /*!< array to store results from numerical integration */

  double* yy;
  double* dyout;
  double* wt;   /*< This array holds computed weights used to estimate
                    correction error limits. */
  double* p;    /*< This is the array de passes to step to use for
                   predicted and corrected states. If successful, step
                   copies p into its local y vector and returns it as yy
                   to de. If unsuccessful, step just returns its latest y
                   to de as yy. */
  double* yerr; /*<! Array of estimated integration errors passed
                   to fcn. May be used for debugging purposes. */
  double** phi; /*<! Internal array used to set up polynomials. */

  int stepCount; /*! approximate number of steps in ydata array, found by
    tspan / tstep */
  char* outFile; /*!< filename to write results to */
  int neqn;      /*!< number of equations for dynamics */
  int neqn2;     /*!< total number of equations including additional state
        variables that must be integrated for force modules */
  double t;      /*!< current value of the independent variable */
  double* tdata; /*!< saved reported values of t, equal to number of entries
                      in ydata / neqn */
  double tout;   /*!< the desired value of t of simulation output */
  int iflag;     /*!<  indicates the status of integration.
        On input, IFLAG is normally 1 (or -1 in the special case where TOUT is
        not to be exceeded.)  On normal output, IFLAG is 2.  Other output values
        are:
        * 3, integration did not reach TOUT because the error tolerances were
             too small.
             But RELERR and ABSERR were increased appropriately for continuing;
        * 4, integration did not reach TOUT because more than 500 steps were taken;
        * 5, integration did not reach TOUT because the equations appear to be
             stiff;
        * 6, invalid input parameters (fatal error).
        The value of IFLAG is returned negative when the input value is negative
        and the integration does not reach TOUT. */
  bool newtonRaphsonFailure;

  int iterationCount;
  s2_stack_t* reconfigurations; /*!< records time step indices of a full
  factor event during a running simulation */

  double twou;  /*!< u is based on the machine unit roundoff error. it is the
                     smallest positive number such that 1.0+u .gt. 1.0.
                     the user must calculate u and assign twou = 2.0*u and
                     fouru=4.0*u in the following statements before
                     calling the code.  */
  double fouru; /*!< u is based on the machine unit roundoff error. it is the
                    smallest positive number such that 1.0+u .gt. 1.0.
                    the user must calculate u and assign twou = 2.0*u and
                    fouru=4.0*u in the following statements before
                    calling the code. */
  double told;

  double hold; /*!< Previous time stepsize used by de and step */
  double h;    /*!< the nominal step size indicating direction of integration
                    and maximum size of step.  H must be a variable, not a
                    constant */
  double hh;
  double eps;       /*!< local error tolerance per step, must be variable, not a
                         constant */
  double hmax;      /*!< Maximum allowed integration stepsize. */
  double x;         /*!< Time variable passed by de to and changed by step */
  double relerr;    /*!< relative error */
  double abserr;    /*!< absolute error */
  double alpha[13]; /*!< Internal array used to set up polynomials. */
  double beta[13];  /*!< Internal array used to set up polynomials. */
  double psi[13];   /*!< Internal array used to set up polynomials. */
  double sig[14];   /*!< Internal array used to set up polynomials. */
  double w[14];     /*!< Internal array used to set up polynomials. */
  double v[13];     /*!< Internal array used to set up polynomials. */
  double g[14];     /*!< Internal array used to set up polynomials. */
  double gstr[14];  /*!< Internal array used to set up polynomials. */
  double two[14];   /*!< Internal array used to set up polynomials. */
  double rho[14];   /*!< Internal array used to set up polynomials. */
  double stabilityLimits[17]; /*!< stability limits, used to set
    adaptive omega_n */
  double safetyFactor;

  int ns; /* Internal counter of the number of successful
           * solutions. It is used to help set the current
           * integration order. */
  // int isnold;
  int ifail;   /*<! Internal status flag that indicates integrator
                    predition failure. Four failures and the integrator
                    resets the order k to 1. */
  bool phase1; /*<! phase1 is a logical variable. When TRUE all lights
                   are go and the integrator may increase order and
                   integration stepsize. When false, integrator may
                   reduce order and stepsize. Also when false, integrator
                   will move cautiously as its error estimates may be
                   unreliable. */
  bool nornd;  /* nornd is a logical variable. When TRUE a reliable
                * estimate of roundoff error is not available to do
                * not use it. When false, compute estimate of roundoff
                * error and use it to increase prediction accuracy. */
  bool crash;  /*!< set to true if no step can be taken */
  bool start;  /* start is a logical variable. When TRUE this is the
                * first call to de. On subsequent calls deStart is FALSE. */
  int k;       /*!< the appropriate order for the next step */
  int kold;    /*!< the order used on the last successful step */
  int iconv;   /*!< Integrator status flag provided to fcn.
                    iconv = 1 when fcn is called after a successful
                    correction. In this case the states are accurate.
                    iconv = 0 when fcn is called after a prediction
                    following a successful correction.
                    iconv = -1 when fcn is called after a prediction
                    following an unsuccessful correction. */
  int step;    /*!< current time step of numerical integration */
  double baumgarteOmegaN2;
  double baumgarte2OmegaN;
  double smoothingFilter;
  double* integralError; /*!< integral of error, equal to number of
    constraints */
  double integralGain;   /*!< integral gain */

} s2_numericalintegrator_ode113b_t;

void s2_context_allocOde113b(s2_context_t*);

void s2_context_freeOde113b(s2_context_t*);

void s2_ode113b_fcn(double y[], s2_context_t*);

void s2_ode113b_step(void f(double y[], s2_context_t* context),
                     s2_context_t* context);

void s2_ode113b_de(void f(double y[], s2_context_t* context),
                   s2_context_t* context);

void s2_context_integrateEquationsOde113b(s2_context_t* context);

#endif
