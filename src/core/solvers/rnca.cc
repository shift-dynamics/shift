#include "shift-core/solvers/rnca.h"
#include "shift-core/factory.h"

bool Shift::Solver::RNCA::registered_ =
  Shift::Factory<Shift::Solver::SolverBase>::get()->registerModule(
    "rnca", Shift::Solver::RNCA::create);

Shift::Solver::RNCA* Shift::Solver::RNCA::create() {
  return new Shift::Solver::RNCA;
}

void Shift::Solver::RNCA::addKeys() {

  Shift::Solver::SolverBase::addKeys();

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
