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

  q_count_ = context_->qCount();
  dq_count_ = context_->dqCount();

  has_mass_.resize(context_->nonRigidJointCount());
  std::fill(has_mass_.begin(), has_mass_.end(), false);

  has_mass_dq_.resize(dq_count_);
  std::fill(has_mass_dq_.begin(), has_mass_dq_.end(), false);

  gamma_d_.resize(dq_count_);
  std::fill(gamma_d_.begin(), gamma_d_.end(), 0);

  Ml_size_ = ((dq_count_ + 1) * (dq_count_ + 2)) / 2;

  Ml_.resize(Ml_size_);
  std::fill(Ml_.begin(), Ml_.end(), 0);

  Mls_.resize(Ml_size_);
  std::fill(Mls_.begin(), Mls_.end(), 0);

  Mlb_.resize(Ml_size_);
  std::fill(Mlb_.begin(), Mlb_.end(), false);

  Mlbs_.resize(Ml_size_);
  std::fill(Mlbs_.begin(), Mlbs_.end(), false);
}

/*! setup and solve the equations of motion given a vector of states y[]

y[] contains the generalized coordinates q[], the first time derivatives dq[],
and any states required for force modules

y = [q, dq, q_f]
dy = [dq, ddq, dq_f]
*/
void Shift::Solver::RNCA::solve(const double y[], double dy[]) {

  const double* q = y;
  const double* dq = y + q_count_;

  assert(context_);
  context_->updateDHVA(q, dq);
  context_->evaluateConstraints();

  context_->setForces(y);

  setEquations(y);

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

  factorAndSolve();

  if(context_->newtonRaphsonFailure()) {
    return;
  }

  /* verify that multiplying coefficient matrix with lhs gets back the
  right hand side, comment out once verified */
  // s2_solve_verify(context, Ml, fl);

  /* convert angular velocities to time derivative of quaternions
    copy by value all other dq terms to dy array */
  context_->dqTody(dy);

  /*
    copy context->ddq to second half of dy, starting at qCount
  */
  memcpy(dy + q_count_, context_->ddq().data(), dq_count_ * sizeof(double));
}
