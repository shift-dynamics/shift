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
