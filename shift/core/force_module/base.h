#ifndef SHIFT_FORCEMODULE_BASE_H_
#define SHIFT_FORCEMODULE_BASE_H_

#include <functional>
#include <map>

#include "shift/core/base.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {

class Context;

namespace force_module {

/*!
@brief shift::core::Frame abstract base class
*/
class Base
  : public shift::core::Base
  , public parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name)
    , parameter::List() {
    DeclareParameters();
  }
  Base()
    : shift::core::Base("")
    , parameter::List() {
    DeclareParameters();
  }
  virtual ~Base() {}

  static const std::string get_array_key() {
    return "force_modules";
  }
  
  static const std::string get_default_type() {
    return "";
  }
  
  unsigned int get_equation_count() {
    return equation_count_;
  }

  const std::vector<double>& get_q0() {
    return q_0_;
  }

  /*! \brief index of the first internal state in the state vector y, see
   * Context::InitializeAuxiliaryStates */
  void set_state_offset(size_t offset) {
    state_offset_ = offset;
  }

  virtual void ActivateObjects(bool parent_is_active) override;
  

  virtual void Load();
  virtual void Connect() = 0;
  /*! \brief add the module's forces to the context and set the time
   * derivatives of its internal states
   * \param y [in] state vector [q, dq, auxiliary states]
   * \param dy [out] state time derivatives */
  virtual void Service(const double* y, double* dy) = 0;

protected:
  std::string type_;
  virtual void DeclareParameters() override;
  unsigned int equation_count_ = 0;
  std::vector<double> q_0_; /*!< vector of state initial conditions */
  size_t state_offset_ = 0;  /*!< index of the first internal state in y */
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace force_module
}
} // namespace shift

#endif
