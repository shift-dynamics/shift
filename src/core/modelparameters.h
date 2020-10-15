#ifndef SHIFT_MODELPARAMETERS_H_
#define SHIFT_MODELPARAMETERS_H_

#include <string>
#include <vector>

#include "ksl/vec3.h"

#include "shift/core/base.h"
#include "shift/core/integrator/base.h"
#include "shift/core/logger/base.h"
#include "shift/core/parameter/list.h"
#include "shift/core/solver/base.h"
#include "shift/core/units.h"

namespace shift {
namespace core {

class Context;

/*!
\brief Shift Assembly datastructure
*/
class ModelParameters
  : public Base
  , public parameter::List {

public:
  ModelParameters(Context* context)
    : Base("") {
    context_ = context;
    addParameters();
  }

  ModelParameters()
    : Base("") {
    addParameters();
  }

  ~ModelParameters() {}

  /*! \brief retrieve gravity vector */
  const ksl_vec3_t* gravity() const {
    return &gravity_;
  }

  /*! \brief allocate dynamic memory in solvers, integrators and loggers */
  void initialize();

  /*! \brief set gravity vector */
  void setGravity(ksl_vec3_t);

  /*! \brief set gravity vector */
  void setGravity(double, double, double);

  /*! \brief get epsilon used for rank determination in Gaussian Elimination
   * with complete row and column pivoting */
  double epsilonRankDetermination() const {
    return epsilon_rank_determination_;
  }

  double epsilsonNewtonRaphson() const {
    return epsilson_newton_raphson_;
  }

  /*! \brief get infinity norm multiplier -- used as a heuristic for when to
   * reapply gaussian elimination with complete row and column pivoting */
  double infinityNormMultiplier() const {
    return infinity_norm_muliplier_;
  }

  void enableContactDetection(bool);

  void appendSolver(std::unique_ptr<solver::Base>&);
  void appendIntegrator(std::unique_ptr<integrator::Base>&);
  void appendLogger(std::unique_ptr<logger::Base>&);

  FILE* debugFilePointer() const {
    return debug_pointer_;
  }

private:
  virtual void addParameters() override;

  std::string version_;
  std::vector<std::string> include_directories_;
  ksl_vec3_t gravity_ = {{0.0, 0.0, 0.0}};
  double epsilson_newton_raphson_ = 1e-7;
  double epsilon_rank_determination_ = 1e-9;
  double infinity_norm_muliplier_ = 2.0;

  bool contact_detection_enabled_ = false;
  ksl_vec3_t viewport_background_color_ = {{1.0, 1.0, 1.0}};

  parameter::FactoryArray<solver::Base> solvers_;
  parameter::FactoryArray<integrator::Base> integrators_;
  parameter::FactoryArray<logger::Base> loggers_;
  Units units_;

  FILE* debug_pointer_ = nullptr;
};

} // namespace core
} // namespace shift

#endif
