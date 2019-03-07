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

  SerialEntry multiplier("infinity_norm_multiplier", SerialType::DOUBLE,
                         &infinity_norm_multiplier_);
  multiplier.setRequired(false);
  multiplier.setDescription(
    "The infinity norm of the B matrix associated with the Jacobian matrix of "
    "each kinematic substructure is used as a heuristic to determine when to "
    "reapply Generalized Coordinate Partitioning. The infinity norm (largest "
    "absolute value of any term) in the B matrix typically starts at 1 after "
    "GCP is initially applied and increases as the optimal split of "
    "independent and dependent variables changes. Setting this value to a "
    "value slightly higher than one will cause GCP to be applied more often. "
    "Likewise, setting the value to a higher value such as 4 or 5 will cause "
    "GCP to be called less often. Default: 2");
  addSerialEntry(multiplier);
}

void Shift::Solver::RNCA::initialize() {
  assert(context_);

  unsigned int dqCount = context_->getdqCount();

  hasMass_.resize(context_->getNonRigidJointCount());
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
