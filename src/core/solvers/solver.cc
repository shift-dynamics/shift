#include "shift-core/solvers/solver.h"

void Shift::Solver::Factory::registerModule(const std::string& name,
                                            CreateFunction pfnCreate) {
  factoryMap_.insert(std::make_pair(name, pfnCreate));
}
