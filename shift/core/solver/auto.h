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
The rows and columns of the delegated trees, those of their joints and of
the elastic coordinates of their flexible bodies (the velocities of
ArticulatedBody::get_tree_dq), are left out of the RNCA mass matrix, so they
add no operations to its factorization, and the tree solver gives their
accelerations and their kinetic energy; a model without closed
loops is solved entirely by the articulated-body solver, and a model whose
trees all have closed loops entirely by RNCA.

*Implicit integration.* The residual is RNCA's in the rows of its
variables and that of the accelerations of the articulated-body solver in
the rows of the delegated trees (see solver::ArticulatedBody), and so is the
iteration matrix, by rows: with \f$S_r\f$ RNCA's iteration matrix of its
variables and multipliers, \f$A_{rd}\f$ the entries of \f$c\, C + k\, K\f$
in its rows and the columns of the delegated variables, and
\f$G_{dr}\f$, \f$G_{dd}\f$ the rows of the delegated variables,
\f$M_d^{-1}(c\, C + k\, K)\f$ with the identity added in \f$G_{dd}\f$,
\f[
\begin{bmatrix} S_r & A_{rd} \\ G_{dr} & G_{dd} \end{bmatrix}
\begin{Bmatrix} \underline{x}_r \\ \underline{x}_d \end{Bmatrix} =
\begin{Bmatrix} \underline{b}_r \\ \underline{b}_d \end{Bmatrix}
\f]
is solved with the Schur complement of the delegated block,
\f$Z = G_{dd} - G_{dr} W\f$, \f$W = S_r^{-1} A_{rd}\f$, factored densely.
The trees are coupled only by forces between them; without such forces
\f$W\f$ is zero.
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

  /*! \brief implicit integration is supported, the delegated trees by the
   * tree solver, flexible bodies included */
  virtual bool supports_implicit() const override;

  /*! \brief RNCA's residual, with the residual of the accelerations of the
   * delegated trees in their rows (see the class documentation) */
  virtual bool Residual(const double t, const std::vector<double>& y,
                        const std::vector<double>& ddq,
                        const std::vector<double>& la, std::vector<double>& r,
                        std::vector<double>& dy) override;

  /*! \brief RNCA's iteration matrix of its own variables, and the rows of
   * the delegated trees and the coupling (see the class documentation), at
   * the state of the last Residual */
  virtual bool FactorIterationMatrix(const Tangents& tangents, double c,
                                     double k,
                                     const std::vector<int>& differenced,
                                     const std::vector<double>& k_columns,
                                     const std::vector<double>& c_columns)
    override;

  virtual bool SolveIterationMatrix(std::vector<double>& rhs) override;

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

  /* the iteration matrix: the delegated variables (the articulated-body
  solver's get_tree_dq()); G_dr, their rows in the columns of RNCA's
  variables (rows by dq_count, row major); W = S_r^-1 A_rd, by columns of
  the size of RNCA's system; and the LU factors of Z */
  std::vector<int> implicit_dq_;
  std::vector<double> rows_dr_;
  std::vector<double> coupling_;
  std::vector<double> schur_;
  std::vector<int> schur_pivots_;
  bool has_coupled_matrix_ = false;

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
