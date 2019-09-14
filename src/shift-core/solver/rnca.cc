#include <assert.h>

#include "shift-core/context.h"
#include "shift-core/factory.h"
#include "shift-core/solver/rnca.h"

bool Shift::Solver::RNCA::registered_ =
  Shift::Factory<Shift::Solver::Base>::get()->registerModule(
    "rnca", Shift::Solver::RNCA::create);

std::unique_ptr<Shift::Solver::RNCA> Shift::Solver::RNCA::create() {
  return std::make_unique<Shift::Solver::RNCA>();
}

void Shift::Solver::RNCA::addParameters() {
  Shift::Solver::Base::addParameters();
}

void Shift::Solver::RNCA::initialize() {
  assert(context_);

  unsigned int dqCount = context_->dqCount();

  hasMass_.resize(context_->nonRigidJointCount());
  hasMassdq_.resize(dqCount);

  if(gamma_d_) {
    delete[] gamma_d_;
    gamma_d_ = NULL;
  }
  gamma_d_ = new double[dqCount]{};

  Ml_size_ = ((dqCount + 1) * (dqCount + 2)) / 2;

  if(Ml_) {
    delete[] Ml_;
    Ml_ = NULL;
  }
  Ml_ = new double[Ml_size_]{};

  if(Mls_) {
    delete[] Mls_;
    Mls_ = NULL;
  }
  Mls_ = new double[Ml_size_]{};

  if(Mlb_) {
    delete[] Mlb_;
    Mlb_ = NULL;
  }
  Mlb_ = new bool[Ml_size_]{};

  if(Mlbs_) {
    delete[] Mlbs_;
    Mlbs_ = NULL;
  }
  Mlbs_ = new bool[Ml_size_]{};
}

void Shift::Solver::RNCA::solve() {

#if 0
  /*!
  @brief solve for current time step
  */
  void s2_ode113b_fcn(double y[]) {

    iteration_count_++;

    assert(context_);
    memcpy(context->q(), y,
           (context_->qCount() + context_->dqCount()) * sizeof(double));
    context_->updateDHVA();
    context_->evaluateConstraints();
    context_->setForces(y);
    context_->setEquations();

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

    context_->factorAndSolve();

    if(newton_raphson_failure_) {
      return;
    }

    /* verify that multiplying coefficient matrix with lhs gets back the
    right hand side, comment out once verified */
    // s2_solve_verify(context, Ml, fl);

    /* convert angular velocities to time derivative of quaternions
      copy by value all other dq terms to dy array */
    context_->dqTody(dy_);

    /*
      copy context->ddq to second half of dy, starting at qCount
    */
    memcpy(numericalIntegrator->dy + context->qCount, context->ddq,
           context->dqCount * sizeof(double));

    // printf("time: %0.16g\n", numericalIntegrator->x);
#endif
}
