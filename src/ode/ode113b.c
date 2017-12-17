#include "ode113b.h"
#include "../force_modules/tire.h"

/*!
@brief allocate memory for ode113b numerical integrator
*/
void s2_ode113b_allocate(
  s2_context_t* context) {

  context->numericalIntegrator =
    malloc(sizeof(s2_numericalintegrator_ode113b_t));

  s2_numericalintegrator_ode113b_t* numericalIntegrator =
    (s2_numericalintegrator_ode113b_t*)context->numericalIntegrator;

  numericalIntegrator->neqn =
    context->qCount + context->dqCount;

  for(int i = 0; i < context->forceModuleCount; i++) {
    numericalIntegrator->neqn += context->forceModulePointers[i]->neqn;
  }

  /*
    allocate and set initial conditions for y array

    y = {q }
        {dq}
  */
  numericalIntegrator->y =
    malloc(numericalIntegrator->neqn * sizeof(double));

  /* copy initial conditions from dynamics */
  memcpy(
    numericalIntegrator->y,
    context->q,
    (context->qCount + context->dqCount) * sizeof(double));

  /* copy initial conditions from force modules */
  int i = context->qCount + context->dqCount;
  for(int j = 0; j < context->forceModuleCount; j++) {
    s2_forcemodule_t* forceModule = context->forceModulePointers[j];
    forceModule->yOffset = i;
    for(int k = 0; k < forceModule->neqn; k++, i++) {
      // printf(
      //   "force module [%d], copying ydata to [%d], value: %f\n",
      //   j,
      //   i,
      //   forceModule->y[k]);
      numericalIntegrator->y[i] = forceModule->y[k];
    }
  }



  numericalIntegrator->stepCount = ceil(
                                     (double)context->modelParameters.timeSpan / (double)context->modelParameters.timeStep) + 1;
  numericalIntegrator->ydata =
    calloc(
      numericalIntegrator->neqn *
      numericalIntegrator->stepCount,
      sizeof(double));
  numericalIntegrator->tdata =
    calloc(
      numericalIntegrator->stepCount,
      sizeof(double));
  memcpy(
    numericalIntegrator->ydata,
    numericalIntegrator->y,
    numericalIntegrator->neqn * sizeof(double));

  numericalIntegrator->dy =
    calloc(numericalIntegrator->neqn, sizeof(double));

  s2_forcemodule_connect(context);

  numericalIntegrator->yy =
    calloc(numericalIntegrator->neqn, sizeof(double));
  memcpy(
    numericalIntegrator->yy,
    numericalIntegrator->y,
    numericalIntegrator->neqn * sizeof(double));

  numericalIntegrator->dyout =
    calloc(numericalIntegrator->neqn, sizeof(double));

  numericalIntegrator->wt =
    calloc(numericalIntegrator->neqn, sizeof(double));

  numericalIntegrator->p =
    calloc(numericalIntegrator->neqn, sizeof(double));

  numericalIntegrator->yerr =
    calloc(numericalIntegrator->neqn, sizeof(double));

  numericalIntegrator->phi =
    s2_array_malloc2D(17, numericalIntegrator->neqn);

  numericalIntegrator->iflag = 1;
  numericalIntegrator->newtonRaphsonFailure = false;
  numericalIntegrator->t = 0;
  numericalIntegrator->x = 0.0;
  numericalIntegrator->tout = 0.0;
  numericalIntegrator->reconfigurations = s2_stack();
  numericalIntegrator->iterationCount = 0;
  numericalIntegrator->hmax = 0.01;

  /*
  The only machine dependent constants are based on the machine unit
  roundoff error  u  which is the smallest positive number such that
  1.0+u .gt. 1.0  .  the user must calculate  u  and assign
  twou = 2.0 * u  and  fouru = 4.0 * u in the following statements before
  calling the code.
  */
  numericalIntegrator->twou = 1.42e-14;
  numericalIntegrator->fouru = 2.84e-14;
  numericalIntegrator->told = 0.0;
  numericalIntegrator->hold = 0.0;
  memset(numericalIntegrator->beta, 0, 13 * sizeof(double));
  memset(numericalIntegrator->psi, 0, 13 * sizeof(double));
  memset(numericalIntegrator->alpha, 0, 13 * sizeof(double));
  memset(numericalIntegrator->sig, 0, 14 * sizeof(double));
  numericalIntegrator->sig[1] = 1.0;
  memset(numericalIntegrator->w, 0, 14 * sizeof(double));
  memset(numericalIntegrator->v, 0, 13 * sizeof(double));
  memset(numericalIntegrator->g, 0, 14 * sizeof(double));
  numericalIntegrator->g[1] = 1.0;
  numericalIntegrator->g[2] = 0.5;
  memset(numericalIntegrator->gstr, 0, 14 * sizeof(double));
  numericalIntegrator->gstr[0] = 0.0;
  numericalIntegrator->gstr[1] = 0.5;
  numericalIntegrator->gstr[2] = 0.0833;
  numericalIntegrator->gstr[3] = 0.0417;
  numericalIntegrator->gstr[4] = 0.0264;
  numericalIntegrator->gstr[5] = 0.0188;
  numericalIntegrator->gstr[6] = 0.0143;
  numericalIntegrator->gstr[7] = 0.0114;
  numericalIntegrator->gstr[8] = 0.00936;
  numericalIntegrator->gstr[9] = 0.00789;
  numericalIntegrator->gstr[10] = 0.00679;
  numericalIntegrator->gstr[11] = 0.00592;
  numericalIntegrator->gstr[12] = 0.00524;
  numericalIntegrator->gstr[13] = 0.00468;
  memset(numericalIntegrator->two, 0, 14 * sizeof(double));
  numericalIntegrator->two[0] = 0.0;
  numericalIntegrator->two[1] = 2.0;
  numericalIntegrator->two[2] = 4.0;
  numericalIntegrator->two[3] = 8.0;
  numericalIntegrator->two[4] = 16.0;
  numericalIntegrator->two[5] = 32.0;
  numericalIntegrator->two[6] = 64.0;
  numericalIntegrator->two[7] = 128.0;
  numericalIntegrator->two[8] = 256.0;
  numericalIntegrator->two[9] = 512.0;
  numericalIntegrator->two[10] = 1024.0;
  numericalIntegrator->two[11] = 2048.0;
  numericalIntegrator->two[12] = 4096.0;
  numericalIntegrator->two[13] = 8192.0;
  memset(numericalIntegrator->rho, 0, 14 * sizeof(double));
  numericalIntegrator->rho[1] = 1.0;
  numericalIntegrator->crash = false;
  numericalIntegrator->start = false;
  numericalIntegrator->k = 0;
  numericalIntegrator->kold = 0;
  numericalIntegrator->iconv = 0;
  numericalIntegrator->step = 0;
  numericalIntegrator->stabilityLimits[0] = 1.0; /* no data for k = 0 */
  numericalIntegrator->stabilityLimits[1] = 2.0;
  numericalIntegrator->stabilityLimits[2] = 2.26;
  numericalIntegrator->stabilityLimits[3] = 1.86;
  numericalIntegrator->stabilityLimits[4] = 1.4;
  numericalIntegrator->stabilityLimits[5] = 1.03;
  numericalIntegrator->stabilityLimits[6] = 0.78;
  numericalIntegrator->stabilityLimits[7] = 0.58;
  numericalIntegrator->stabilityLimits[8] = 0.44;
  numericalIntegrator->stabilityLimits[9] = 0.34;
  numericalIntegrator->stabilityLimits[10] = 0.26;
  numericalIntegrator->stabilityLimits[11] = 0.21;
  numericalIntegrator->stabilityLimits[12] = 0.6;
  numericalIntegrator->stabilityLimits[13] = 0.2; /* no data for k = 13 */
  numericalIntegrator->stabilityLimits[14] = 0.2; /* no data for k = 14 */
  numericalIntegrator->stabilityLimits[15] = 0.2; /* no data for k = 15 */
  numericalIntegrator->stabilityLimits[16] = 0.2; /* no data for k = 16 */
  numericalIntegrator->safetyFactor = 0.005;
  numericalIntegrator->baumgarte2OmegaN =
    context->modelParameters.baumgarte2OmegaN;
  numericalIntegrator->baumgarteOmegaN2 =
    context->modelParameters.baumgarteOmegaN2;
  numericalIntegrator->smoothingFilter = 0.001;

  numericalIntegrator->integralGain = 1000.0;

  numericalIntegrator->integralError =
    calloc(context->chordConstraintCount * 6, sizeof(double));

}


/*!
@brief free memory for ode113b integrator
*/
void s2_ode113b_free(
  s2_context_t* context) {

  s2_numericalintegrator_ode113b_t* numericalIntegrator =
    (s2_numericalintegrator_ode113b_t*)context->numericalIntegrator;

  s2_array_free(numericalIntegrator->y);
  s2_array_free(numericalIntegrator->ydata);
  s2_array_free(numericalIntegrator->tdata);
  s2_array_free(numericalIntegrator->dy);
  s2_array_free(numericalIntegrator->yy);
  s2_array_free(numericalIntegrator->dyout);
  s2_array_free(numericalIntegrator->wt);
  s2_array_free(numericalIntegrator->p);
  s2_array_free(numericalIntegrator->yerr);
  s2_array_free2D(numericalIntegrator->phi);

  free(numericalIntegrator);
  context->numericalIntegrator = NULL;
}


/*!
@brief solve for current time step
*/
void s2_ode113b_fcn(
  double y[],
  s2_context_t* context) {

  /*
    copy y to context->q and context->dq

    this copy by value would not be necessary if the setEquations
    function was designed to work with relative memory pointers or indices

    candidate for future speed improvement
  */
  s2_numericalintegrator_ode113b_t* numericalIntegrator =
    (s2_numericalintegrator_ode113b_t*)context->numericalIntegrator;
  numericalIntegrator->iterationCount++;

  memcpy(
    context->q,
    y,
    (context->qCount + context->dqCount) * sizeof(double));

  s2_updateDHVA(context);
  s2_jacobian_evaluateConstraints(context);
  s2_service_setForces(context, y);
  s2_equation_setEquations(context);

  /* make a copy of Ml and fl to verify equations of motion, remove this
  once equations of motion verified */
  // double Ml[context->dqCount * context->dqCount];
  // for(int i = 0; i < context->nonRigidJointCount; i++) {
  //   int istart = i * (i + 1) / 2;
  //   for(int j = 0; j < i + 1; j++) {
  //     Ml[i * context->nonRigidJointCount + j] = context->Ml[istart + j];
  //     if(i != j) {
  //       Ml[j * context->nonRigidJointCount + i] = context->Ml[istart + j];
  //     }
  //   }
  // }

  /* copy fl to a second array to verify solution of
  equations of motion, uncomment for debugging */
  // double fl[context->dqCount];
  // memcpy(fl, context->fl, context->dqCount * sizeof(double));

  s2_equation_factorAndSolve(context);

  if(numericalIntegrator->newtonRaphsonFailure) {
    return;
  }

  /* verify that multiplying coefficient matrix with lhs gets back the
  right hand side, comment out once verified */
  // s2_solve_verify(context, Ml, fl);

  /*
    convert angular velocities to time derivative of quaternions
    copy by value all other dq terms to dy array
  */
  s2_equation_dq_to_dy(context, numericalIntegrator->dy);

  /*
    copy context->ddq to second half of dy, starting at qCount
  */
  memcpy(
    numericalIntegrator->dy + context->qCount,
    context->ddq,
    context->dqCount * sizeof(double));

  // printf("time: %0.16g\n", numericalIntegrator->x);

  if(numericalIntegrator->iconv == 1) {

    int k = numericalIntegrator->k;

    double h = numericalIntegrator->x - numericalIntegrator->t;

    h = fmax(h, 0.00001); /* limit step size */

    double* SL = numericalIntegrator->stabilityLimits;

    double safetyFactor = numericalIntegrator->safetyFactor;

    double omegaN = SL[k] / h * safetyFactor;

    double omegaNl = numericalIntegrator->baumgarte2OmegaN / 2;

    omegaN = fmin(omegaN, 1000); /* limit omegaN */

    omegaN = omegaNl + numericalIntegrator->smoothingFilter *
             (omegaN - omegaNl);

    numericalIntegrator->baumgarte2OmegaN = 2 * omegaN;
    numericalIntegrator->baumgarteOmegaN2 = omegaN * omegaN;

    double rhodt = h * numericalIntegrator->integralGain;
    for(int i = 0; i < context->chordConstraintCount * 6; i++) {
      numericalIntegrator->integralError[i] +=
        context->cs[i] * rhodt;
    }

    numericalIntegrator->t = numericalIntegrator->x;
    s2_forcemodule_saveHistory(context);

    if(numericalIntegrator->x >=
        numericalIntegrator->tdata[numericalIntegrator->step] + context->modelParameters.timeStep) {
      s2_solve_writeStates(
        context, ++(numericalIntegrator->step));
      // printf("\tk: %d, h: %0.9g, SL: %f, safetyFactor: %f, omegaN: %f\n", k, h, SL[k], safetyFactor, omegaN);
      numericalIntegrator->iterationCount = 0;

      /* write dhva information to dhva debug file */
      // FILE* dhva = context->modelParameters->dhvaFilePointer;
      // s2_updateDHVADebug(dhva, context, numericalIntegrator->t);
    }
  }
  numericalIntegrator->iconv = 0;
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
void s2_ode113b_step(
  void f(double y[], s2_context_t* context),
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
  numericalIntegrator->h = fmax(numericalIntegrator->h, numericalIntegrator->fouru);
  p5eps = 0.5 * numericalIntegrator->eps;

  /*  if error tolerance is too small, increase it to an acceptable value */
  round = 0.0;
  for(int l = 0; l < numericalIntegrator->neqn; l++) {
    round += pow(numericalIntegrator->yy[l] / numericalIntegrator->wt[l], 2);
  }
  round = numericalIntegrator->twou * sqrt(round);
  if(p5eps < round) {
    numericalIntegrator->eps = 2.0 * round * (1.0 + numericalIntegrator->fouru);
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
    if(numericalIntegrator->eps < 16.0 * sum * numericalIntegrator->h * numericalIntegrator->h) {
      numericalIntegrator->h = 0.25 * sqrt(numericalIntegrator->eps / sum);
    }

    numericalIntegrator->h =
      fmax(
        numericalIntegrator->h,
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
          numericalIntegrator->beta[i] =
            numericalIntegrator->beta[im1] *
            numericalIntegrator->psi[im1] / temp2;
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
            for(int j = 1; j <= nsm2; j++)  {
              int i = numericalIntegrator->k - j;
              numericalIntegrator->v[i] =
                numericalIntegrator->v[i] -
                numericalIntegrator->alpha[j + 1] *
                numericalIntegrator->v[i + 1];
            }
          }
        }
        /*  update v(*) and set w(*) */
        limit1 = kp1 - numericalIntegrator->ns;
        temp5 = numericalIntegrator->alpha[numericalIntegrator->ns];
        for(iq = 1; iq <= limit1; iq++) {
          numericalIntegrator->v[iq] -= temp5 * numericalIntegrator->v[iq + 1];
          numericalIntegrator->w[iq] = numericalIntegrator->v[iq];
        }
        numericalIntegrator->g[nsp1] = numericalIntegrator->w[1];
      }
      /*  compute g(*) in the work vector w(*) */
      nsp2 = numericalIntegrator->ns + 2;
      if(kp1 >= nsp2) {
        for(int i = nsp2; i <= kp1; i++) {
          for(iq = 1; iq <= kp2 - i; iq++) {
            numericalIntegrator->w[iq] -=
              numericalIntegrator->alpha[i - 1] *
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
            numericalIntegrator->beta[i] *
            numericalIntegrator->phi[i][l];
        }
      }
    }

    /*  predict solution and differences */
    for(int l = 0; l < numericalIntegrator->neqn; l++) {
      numericalIntegrator->phi[kp2][l] =
        numericalIntegrator->phi[kp1][l];
      numericalIntegrator->phi[kp1][l] = 0.0;
      numericalIntegrator->p[l] = 0.0;
    }
    for(int j = 1; j <= numericalIntegrator->k; j++) {
      int i = kp1 - j;
      ip1 = i + 1;
      for(int l = 0; l < numericalIntegrator->neqn; l++) {
        numericalIntegrator->p[l] +=
          numericalIntegrator->g[i] *
          numericalIntegrator->phi[i][l];
        numericalIntegrator->phi[i][l] +=
          numericalIntegrator->phi[ip1][l];
      }
    }
    if(numericalIntegrator->nornd) {
      for(int l = 0; l < numericalIntegrator->neqn; l++)
        numericalIntegrator->p[l] =
          numericalIntegrator->yy[l] +
          numericalIntegrator->h *
          numericalIntegrator->p[l];
    } else {
      for(int l = 0; l < numericalIntegrator->neqn; l++) {
        tau =
          numericalIntegrator->h *
          numericalIntegrator->p[l] -
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
        erkm2 +=
          pow(
            (numericalIntegrator->phi[km1][l] + temp4) / numericalIntegrator->wt[l], 2);
        erkm1 +=
          pow(
            (numericalIntegrator->phi[numericalIntegrator->k][l] + temp4) / numericalIntegrator->wt[l], 2);
        erk += numericalIntegrator->yerr[l];
      }
      erkm2 =
        numericalIntegrator->h * numericalIntegrator->sig[km1] * numericalIntegrator->gstr[km2] * sqrt(erkm2);
      erkm1 =
        numericalIntegrator->h * numericalIntegrator->sig[numericalIntegrator->k] * numericalIntegrator->gstr[km1] * sqrt(erkm1);
    } else if(km1 > 0) {
      for(int l = 0; l < numericalIntegrator->neqn; l++) {
        temp4 = numericalIntegrator->dy[l] - numericalIntegrator->phi[1][l];
        numericalIntegrator->yerr[l] =
          pow(temp4 / numericalIntegrator->wt[l], 2);
        erkm1 += pow(
                   (numericalIntegrator->phi[numericalIntegrator->k][l] + temp4) / numericalIntegrator->wt[l], 2);
        erk += numericalIntegrator->yerr[l];
      }
      erkm1 =
        numericalIntegrator->h * numericalIntegrator->sig[numericalIntegrator->k] * numericalIntegrator->gstr[km1] * sqrt(erkm1);
    } else {
      for(int l = 0; l < numericalIntegrator->neqn; l++) {
        temp4 = numericalIntegrator->dy[l] - numericalIntegrator->phi[1][l];
        numericalIntegrator->yerr[l] =
          pow(temp4 / numericalIntegrator->wt[l], 2);
        erk += numericalIntegrator->yerr[l];
      }
    }
    temp5 = numericalIntegrator->h * sqrt(erk);
    err =
      temp5 * (numericalIntegrator->g[numericalIntegrator->k] -
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
        numericalIntegrator->phi[i][l] =
          (numericalIntegrator->phi[i][l] - numericalIntegrator->phi[ip1][l]) / numericalIntegrator->beta[i];
      }
    }

    if(numericalIntegrator->k >= 2) {
      for(int i = 2; i <= numericalIntegrator->k; i++) {
        numericalIntegrator->psi[i - 1] =
          numericalIntegrator->psi[i] - numericalIntegrator->h;
      }
    }

    /* on third failure, set order to one. thereafter, use optimal step size */
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
    if(
      numericalIntegrator->h < numericalIntegrator->fouru * numericalIntegrator->x) {
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
        numericalIntegrator->p[l] + temp1 *
        (numericalIntegrator->dy[l] - numericalIntegrator->phi[1][l]);
  } else {
    for(int l = 0; l < numericalIntegrator->neqn; l++) {
      rho = temp1 * (numericalIntegrator->dy[l] - numericalIntegrator->phi[1][l]) - numericalIntegrator->phi[16][l];
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
  } else if(kp1 <= numericalIntegrator->ns)  {
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
  if(
    (!numericalIntegrator->phase1) &&
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
void s2_ode113b_de(
  void f(double y[], s2_context_t* context),
  s2_context_t* context) {

  s2_numericalintegrator_ode113b_t* numericalIntegrator =
    (s2_numericalintegrator_ode113b_t*)context->numericalIntegrator;

  double releps;
  double abseps;

  numericalIntegrator->step = 0;
  numericalIntegrator->eps =
    fmax(
      context->modelParameters.relativeError,
      context->modelParameters.absoluteError);

  /* test for improper parameters */
  if(
    numericalIntegrator->neqn < 1 ||
    context->modelParameters.timeSpan == 0.0 ||
    context->modelParameters.relativeError < 0.0 ||
    context->modelParameters.absoluteError < 0.0 ||
    numericalIntegrator->iflag < 1 ||
    numericalIntegrator->iflag > 5 ||
    numericalIntegrator->eps < numericalIntegrator->twou) {

    printf(
      "neqn=%d, time=%g, timeEnd=%g, relerr=%g, abserr=%g, iflag=%d, eps=%g, " "twou=%g\n",
      numericalIntegrator->neqn,
      0.0,
      context->modelParameters.timeSpan,
      context->modelParameters.relativeError,
      context->modelParameters.absoluteError,
      numericalIntegrator->iflag,
      numericalIntegrator->eps,
      numericalIntegrator->twou);

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
    memcpy(
      numericalIntegrator->yy,
      numericalIntegrator->y,
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
      fmin(
        numericalIntegrator->h,
        context->modelParameters.timeSpan - numericalIntegrator->x);
    for(int l = 0; l < numericalIntegrator->neqn; l++) {
      numericalIntegrator->wt[l] =
        releps * fabs(numericalIntegrator->yy[l]) + abseps;
    }

    s2_ode113b_step(
      f,
      context);

    if(numericalIntegrator->newtonRaphsonFailure) {
      fprintf(stderr, "NR Failure\n");
      return;
    }

    if(numericalIntegrator->x >= context->modelParameters.timeSpan) {
      numericalIntegrator->iflag = 2;
      return;
    }

    /*  If crash, increase error tolerances and return to
      calling program. Can call de again to continue integration with larger tolerances.*/
    if(numericalIntegrator->crash) {
      numericalIntegrator->iflag = 3;
      numericalIntegrator->relerr = numericalIntegrator->eps * releps;
      numericalIntegrator->abserr = numericalIntegrator->eps * abseps;
      memcpy(
        numericalIntegrator->y,
        numericalIntegrator->yy,
        numericalIntegrator->neqn * sizeof(double));
      numericalIntegrator->x = numericalIntegrator->t;
      return;
    } /*End if crash*/
  }   /*End While*/
}


/*!
@brief placeholder for integrating using ode113b method
*/
void s2_ode113b_integrateEquations(
  s2_context_t* context) {

  s2_ode113b_de(s2_ode113b_fcn, context);
  s2_numericalintegrator_ode113b_t* numericalIntegrator =
    (s2_numericalintegrator_ode113b_t*)context->numericalIntegrator;

  /*! if number of steps is less than total amount of space allocated, pad the
  states out to the end of the space allocated to avoid incorrect display in
  graphics

  @TODO need to add interpolation in graphics for data at irregular time
  interval reporting */
  if(numericalIntegrator->step < numericalIntegrator->stepCount - 1) {
    for(
      int i = numericalIntegrator->step;
      i < numericalIntegrator->stepCount;
      i++) {

      memcpy(
        numericalIntegrator->ydata + numericalIntegrator->neqn * i,
        numericalIntegrator->ydata + numericalIntegrator->neqn *
        (numericalIntegrator->step - 1),
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
