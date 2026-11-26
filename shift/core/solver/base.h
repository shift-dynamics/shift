#ifndef SHIFT_SOLVER_BASE_H_
#define SHIFT_SOLVER_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ksl/vec3.h"

#include "shift/core/base.h"
#include "shift/core/parameter/list.h"
#include "shift/core/topology.h"

namespace shift {
namespace core {

namespace body {
class Base;
}

namespace joint {
class Base;
}

namespace constraint {
class Base;
}

namespace integrator {
class Base;
}

class Frame;
class Context;
class Tangents;

namespace solver {

/*!
@brief Abstract Solver base class
*/
class Base
  : public shift::core::Base
  , public shift::core::parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name) {}
  Base()
    : shift::core::Base("") {}
  virtual ~Base() {}

  const std::string& get_type() {
    return type_;
  }

  static const std::string get_array_key() {
    return "solvers";
  }

  static const std::string get_default_type() {
    return "";
  }

  virtual void ActivateObjects(bool parent_is_active) override;
  virtual void Load();
  virtual void Connect();
  virtual void Initialize() = 0;
  virtual double GravitationalPotentialEnergy(shift::core::topology::DHVA&) {
    return 0.0;
  }
  virtual double KineticEnergy(shift::core::topology::DHVA&) {
    return 0.0;
  }
  virtual bool Solve(const double t, const std::vector<double>& y,
                     std::vector<double>& dy) = 0;

  /*! \name Implicit integration
  What the implicit integrators (integrator::GeneralizedAlpha) need beyond
  Solve. They advance all the generalized coordinates, independent and
  dependent, and solve the equations of motion at the end of a step for the
  accelerations and the multipliers by Newton's method, in the solver's own
  form of the equations. For RNCA that form is the preconditioned system of
  RNCA::Factor: the multipliers are the preconditioned multipliers
  \f$\underline{\lambda}^d\f$, one per dependent coordinate (for RNCA in
  the order of the dependent coordinates in permutation order 3), so their
  number and meaning are those of the current partition. Residual does not
  check the partition, so it changes only in Solve, which the integrator
  calls between steps. A solver that does
  not support implicit integration returns false from supports_implicit(),
  and the integrator refuses to start.
  */
  ///@{
  virtual bool supports_implicit() const {
    return false;
  }

  /*! \brief the number of multipliers of the current partition */
  virtual unsigned int get_multiplier_count() const {
    return 0;
  }

  /*! \brief whether the rows of the multipliers in Residual are the
   * preconditioned velocity constraints
   * \f$\tilde{\underline{\dot{\Phi}}} = [B_{di} \; I]\,
   * \underline{\dot{q}}\f$ rather than the position constraints: the
   * stabilized index-2 form, in which the integrator satisfies the position
   * constraints by a projection of the coordinates (integrator::Alpha) */
  virtual void set_velocity_constraints(bool velocity) {
    (void)velocity;
  }

  /*! \brief the multipliers of the last Solve, the starting values of a
   * step's Newton iterations */
  virtual void Multipliers(std::vector<double>& la) const {
    la.clear();
  }

  /*! \brief the residual of the equations of motion at time t and state y
   * (all coordinates and velocities, and the internal states) for the
   * accelerations \p ddq and multipliers \p la, into \p r: first one row
   * per generalized velocity,
   * \f$M \underline{\ddot{q}} + B^T \underline{\lambda} - \underline{f}\f$,
   * then one row per multiplier, the preconditioned position constraint
   * error \f$\tilde{\underline{\Phi}}(\underline{q})\f$ (for RNCA the
   * correction \f$J_d^{-1}\underline{\Phi}\f$ of the dependent coordinates
   * of one Newton-Raphson iteration, Jacobian::EvaluateNewtonRaphsonCorrections,
   * which RemoveConstraintViolations subtracts from them),
   * whose derivative with respect to the dependent coordinates is I and with
   * respect to the independent ones is \f$B_{di}\f$. The modules are
   * serviced at y (Context::SetForces), which sets the rates of the internal
   * states in \p dy. Returns true on failure. */
  virtual bool Residual(const double t, const std::vector<double>& y,
                        const std::vector<double>& ddq,
                        const std::vector<double>& la, std::vector<double>& r,
                        std::vector<double>& dy) {
    (void)t;
    (void)y;
    (void)ddq;
    (void)la;
    (void)r;
    (void)dy;
    return true;
  }

  /*! \brief form and factor the iteration matrix at the state of the last
   * Residual,
   * \f[
   * S = \begin{bmatrix} M + c\, C + k\, K & B^T \\ B & 0 \end{bmatrix}
   * \f]
   * with the tangent damping C and stiffness K of \p tangents (see
   * Tangents) and, for the generalized velocities in \p differenced, the
   * columns of K and C found by finite differences, given by the integrator
   * as dense dq_count by differenced.size() blocks, row major. A column
   * found by differences holds all its terms and takes the place of the
   * tangents in it. A is not
   * symmetric in general (friction, tires), and S is factored as it is. For
   * RNCA S has the structure of the preconditioned equations of motion with
   * M + cC + kK in place of M, and is factored with the pivots of RNCA's
   * factorization, separately from it (see RNCA::FactorIterationMatrix); the
   * entries of K and C between bodies on different branches are fills that
   * M does not have, which join the pattern of that factorization when the
   * pattern of the tangents changes, such as when a contact begins.
   * Returns true on failure. */
  virtual bool FactorIterationMatrix(const Tangents& tangents, double c,
                                     double k,
                                     const std::vector<int>& differenced,
                                     const std::vector<double>& k_columns,
                                     const std::vector<double>& c_columns) {
    (void)tangents;
    (void)c;
    (void)k;
    (void)differenced;
    (void)k_columns;
    (void)c_columns;
    return true;
  }

  /*! \brief solve S x = rhs with the last factored iteration matrix, in
   * place: rhs holds one entry per generalized velocity, then one per
   * multiplier, as the residual. Any number of solves may follow one
   * FactorIterationMatrix, with Residual evaluated in between. Returns true
   * when there is no factored iteration matrix of the current partition. */
  virtual bool SolveIterationMatrix(std::vector<double>& rhs) {
    (void)rhs;
    return true;
  }

  /*! \brief call \p add(row, column, value) with each entry of
   * \f$c\, C + k\, K\f$ of \p tangents between generalized velocities: the
   * entries between velocities, and the blocks between bodies mapped by the
   * influence coefficients of the velocities on the path of each body to the
   * fixed body (Context::GetLoadPath), \f$\underline{\mathbf{h}}_a^T K_{ab}
   * \underline{\mathbf{h}}_b\f$. An entry may come more than once, to be
   * summed. */
  static void ForEachTangentEntry(
    const Context& context, const Tangents& tangents, double c, double k,
    const std::function<void(int row, int column, double value)>& add);
  ///@}

protected:
  virtual void DeclareParameters() override;

  /*! \brief the kinetic energy of the elastic coordinates of the flexible
   * bodies and of their coupling with the reference frames,
   * \f$\underline{\mathbf{v}}_r \cdot M_{be}\underline{\dot{\eta}} +
   * \frac{1}{2}\underline{\dot{\eta}}^T M_{ee}\underline{\dot{\eta}}\f$,
   * which the momentum \f$M_{bb}\underline{\mathbf{v}}_r\f$ of a frame leaves
   * out, at the rates \p dhva was evaluated at (DHVA::dq); of the bodies
   * whose compressed entries are in \p entries, or of all of them when it is
   * nullptr */
  double ElasticKineticEnergy(const topology::DHVA& dhva,
                              const std::vector<int>* entries = nullptr) const;

  std::string type_; //!< descriptive name of Solver type
  shift::core::integrator::Base* integrator_ = nullptr;
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace solver
}
} // namespace shift

#endif
