#ifndef SHIFT_SOLVER_AUTO_H_
#define SHIFT_SOLVER_AUTO_H_

#include <memory>
#include <string>
#include <vector>

#include "shift/core/solver/articulated_body.h"
#include "shift/core/solver/rnca.h"

namespace shift {
namespace core {
namespace solver {

/*!
@brief Solves each tree of the kinematic forest with the algorithm that suits
it

The trees of a model (see topology::Tree) are dynamically independent unless a
chord constraint closes a loop through them. This solver hands every tree
without closed loops, serial chain or branched, to an articulated-body solver
(solver::ABA by default, or solver::DCA, chosen by `tree_solver`), and solves
the trees with closed loops with the constraint Jacobian as solver::RNCA does.
The rows and columns of the delegated trees are left out of the RNCA mass
matrix, so they add no operations to its factorization; a model without closed
loops is solved entirely by the articulated-body solver, and a model whose
trees all have closed loops entirely by RNCA.
*/
class Auto : public RNCA {

public:
  Auto(std::string name)
    : RNCA(name) {
    DeclareParameters();
  }
  Auto()
    : RNCA("") {
    DeclareParameters();
  }
  virtual ~Auto() override {}

  static bool is_registered() {
    return registered_;
  }

  const std::string& get_description() {
    return description_;
  }

  virtual void Initialize() override;

  virtual double
  GravitationalPotentialEnergy(shift::core::topology::DHVA& dhva) override;
  virtual double KineticEnergy(shift::core::topology::DHVA& dhva) override;

  /*! \brief the indices in Context::get_trees() of the trees handed to the
   * articulated-body solver */
  const std::vector<int>& get_delegated_trees() const {
    return delegated_trees_;
  }

protected:
  virtual void DeclareParameters() override;
  virtual bool SolveDelegated() override;

private:
  static bool registered_;
  static std::unique_ptr<Auto> Create();

  std::string tree_solver_type_ = "aba";
  std::unique_ptr<ArticulatedBody> tree_solver_;
  std::vector<int> delegated_trees_;

  const std::string description_ =
    "Automatic selection of the solver for each tree of the model:\n"
    "Every tree of the kinematic forest without closed loops (a serial chain "
    "or a branched tree) is solved by an articulated-body solver, 'aba' by "
    "default or 'dca', and the trees with closed loops by the run-time "
    "optimized natural constraint algorithm (see 'rnca'), whose parameters "
    "this solver also takes.";
};

} // namespace solver
} // namespace core
} // namespace shift

#endif
