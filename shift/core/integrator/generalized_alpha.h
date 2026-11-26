#ifndef SHIFT_INTEGRATOR_GENERALIZED_ALPHA_H_
#define SHIFT_INTEGRATOR_GENERALIZED_ALPHA_H_

#include <memory>
#include <string>

#include "shift/core/integrator/alpha.h"

namespace shift {
namespace core {
namespace integrator {

/*!
@brief Implicit generalized-\f$\alpha\f$ integrator (see Alpha), with the
coefficients of Chung and Hulbert from the spectral radius at infinite
frequency rho_infinity
*/
class GeneralizedAlpha : public Alpha {

public:
  GeneralizedAlpha(std::string name)
    : Alpha(name) {
    DeclareParameters();
  }
  GeneralizedAlpha()
    : Alpha("") {
    DeclareParameters();
  }
  virtual ~GeneralizedAlpha() override {}
  static std::unique_ptr<GeneralizedAlpha> Create();

  static bool is_registered() {
    return registered_;
  }

protected:
  virtual void DeclareParameters() override;
  virtual void SetCoefficients() override;

private:
  double rho_infinity_ = 0.8;
  static bool registered_;
};

} // namespace integrator
} // namespace core
} // namespace shift

#endif
