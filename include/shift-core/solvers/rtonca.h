#ifndef _SHIFT_RTONCASOLVER_H_
#define _SHIFT_RTONCASOLVER_H_

#include <string>

#include "shift-core/solvers/solver.h"

namespace Shift {

namespace Solver {

/*!
@brief Abstract base class for Shift::Solver C++ objects
*/
class RTONCA : SolverBase {

public:
  RTONCA(std::string name)
    : SolverBase(name) {}
  ~RTONCA() {}

  const std::string& getDescription() {
    return description_;
  }

  void addKeys();

private:
  const std::string description_ =
    "Run-Time Optimized Natural Constraint Algorithm (RTONCA):\n"
    "The RTONCA algorithm is well suited to set up and solve the equations "
    "of motion for complex constrained mechanisms with multiple kinematic "
    "chord loops. Chord loops are identified automatically from the JSON "
    "input file. Overlapping clusters of chord loops (i.e. kinematic "
    "substructures), are identified automatically if present in the model, "
    "thereby allowing the constraint Jacobian matrix to be block-diagonalized. "
    "The optimal split of independent and dependent joint variables is "
    "obtained automatically using a heuristic application of Generalized "
    "Coordinate Partitioning (GCP). The equations of motion are solved "
    "using a custom sparse matrix factor and solve algorithm \n"
    "tailored to the unique structure of multibody dynamics equations.\n"
    "\n"
    "Key features include:\n"
    "  * Block diagonalization via kinematic substructuring\n"
    "  * Heuristic application of GCP via monitoring infinity norm\n"
    "  * Custom sparse factor and solve routine tailored to MBD\n"
    "  * Minimize logic and overhead using arrays of function pointers\n"
    "  * Run-time reconfiguration\n"
    "\n"
    "Best suited for:\n"
    "  * Complex constrained mechanisms\n"
    "  * Tree topologies\n"
    "\n"
    "Refer to the folowing publications for additional information:\n"
    "  * Wehage K., Computational Methods for Dynamics and Mobility Analysis "
    "of Multiloop Mechanisms and Robotic Systems, Thesis, December 2016 "
    "`<https://search.proquest.com/openview/"
    "2b96ee91ebc96f840bdada69a21b7cbe>`_\n"
    "\n"
    "  * Wehage K. Ravani B., A Computational Method for Formulation and "
    "Solution of Dynamical Equations for Complex Mechanisms and Multibody "
    "Systems, Proceedings of IDETC/DIE at Cleveland Ohio, August 2017, "
    "`<https://doi.org/10.1115/DETC2017-67766>`_\n"
    "\n"
    "  * Wehage K., Wehage R., Ravani B., Generalized coordinate partitioning "
    "for complex mechanisms based on kinematic substructuring, Mechanism and "
    "Machine Theory 92:464-483, October 2015 "
    "`<https://doi.org/10.1016/j.mechmachtheory.2015.06.006>`_";
};

} // namespace Solver

} // namespace Shift

#endif
