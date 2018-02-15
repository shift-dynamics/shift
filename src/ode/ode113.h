#ifndef _SHIFT_ODE113_H_
#define _SHIFT_ODE113_H_

#include <stdbool.h>

typedef struct s2_stack_t s2_stack_t;
typedef struct s2_context_t s2_context_t;

/*!
@brief Attributes associated with numerical integrator
*/
typedef struct s2_numericalintegrator_ode113_t {
  double* y;     /*!< the current solution */
  double* dy;    /*!< first time derivative of generalized coordinates */
  double* ydata; /*!< array to store results from numerical integration */
  int stepCount; /*! number of steps in ydata array, found by tspan / tstep */
  char* outFile; /*!< filename to write results to */
  int neqn;      /*!< number of equations for dynamics */
  int neqn2;     /*!< total number of equations including additional state
        variables that must be integrated for force modules */
  double t;      /*!< current value of the independent variable */
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
  double* work;  /*!< work[100 + 21 * neqn] */
  int* iwork;    /*!< iwork[5] */
  int iterationCount;
  s2_stack_t*
    reconfigurations; /*!< records time step indices of a full factor event
                         during a running simulation */

} s2_numericalintegrator_ode113_t;

void s2_ode113(void f(double t, double y[], double dy[], s2_context_t* context),
               s2_context_t* context);

void s2_ode113_de(
  void f(double t, double y[], double dy[], s2_context_t* context),
  s2_context_t* context, int neqn, double y[], double* t, double tout,
  double relerr, double abserr, int* iflag, double yy[], double wt[],
  double p[], double yp[], double ypout[], double phi[], double alpha[],
  double beta[], double sig[], double v[], double w[], double g[], int* phase1,
  double psi[], double* x, double* h, double* hold, int* start, double* told,
  double* delsgn, int* ns, int* nornd, int* k, int* kold, int* isnold);

int i4_sign(int i);

void s2_ode113_intrp(double x, double y[], double xout, double yout[],
                     double ypout[], int neqn, int kold, double phi[],
                     double psi[]);

double r8_epsilon(void);

void s2_ode113_step(double* x, double y[],
                    void f(double t, double y[], double dy[],
                           s2_context_t* context),
                    s2_context_t* context, int neqn, double* h, double* eps,
                    double wt[], int* start, double* hold, int* k, int* kold,
                    int* crash, double phi[], double p[], double yp[],
                    double psi[], double alpha[], double beta[], double sig[],
                    double v[], double w[], double g[], int* phase1, int* ns,
                    int* nornd);

void timestamp(void);

void s2_context_allocOde113(s2_context_t* context);
void s2_context_freeOde113(s2_context_t* context);
void s2_context_integrateEquationsOde113(s2_context_t* context);

#endif
