#ifndef SHIFT_INTEGRATOR_HHT_H_
#define SHIFT_INTEGRATOR_HHT_H_

#include <memory>
#include <string>

#include "shift/core/integrator/alpha.h"

namespace shift {
namespace core {
namespace integrator {

/*!
@brief Implicit HHT-\f$\alpha\f$ integrator of Hilber, Hughes and Taylor
(see Alpha): \f$\alpha_m = 0\f$, \f$\alpha_f = -\alpha\f$,
\f$\gamma = \tfrac{1}{2} - \alpha\f$, \f$\beta = \tfrac{1}{4}(1 -
\alpha)^2\f$, with alpha in [-1/3, 0]
*/
class HHT : public Alpha {

public:
  HHT(std::string name)
    : Alpha(name) {
    DeclareParameters();
  }
  HHT()
    : Alpha("") {
    DeclareParameters();
  }
  virtual ~HHT() override {}
  static std::unique_ptr<HHT> Create();

  static bool is_registered() {
    return registered_;
  }

protected:
  virtual void DeclareParameters() override;
  virtual void SetCoefficients() override;

private:
  double alpha_ = -0.1;
  static bool registered_;
};

} // namespace integrator
} // namespace core
} // namespace shift

#endif
