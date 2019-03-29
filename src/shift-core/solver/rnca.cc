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

void Shift::Solver::RNCA::addKeys() {

  Shift::Solver::Base::addKeys();
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
