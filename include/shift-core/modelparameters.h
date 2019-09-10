#ifndef SHIFT_MODELPARAMETERS_H_
#define SHIFT_MODELPARAMETERS_H_

#include <string>
#include <vector>

#include "ksl/vec3.h"

#include "shift-core/base.h"
#include "shift-core/integrator/base.h"
#include "shift-core/logger/base.h"
#include "shift-core/parameter/list.h"
#include "shift-core/solver/base.h"
#include "shift-core/units.h"

namespace Shift {

class Context;
/*!
\brief Shift Assembly datastructure
*/
class ModelParameters
  : public Shift::Base
  , public Shift::Parameter::List {

public:
  ModelParameters(Shift::Context* context)
    : Shift::Base("") {
    context_ = context;
    addParameters();
  }

  ModelParameters()
    : Shift::Base("") {
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

  void appendSolver(std::unique_ptr<Solver::Base>&);
  void appendIntegrator(std::unique_ptr<Integrator::Base>&);
  void appendLogger(std::unique_ptr<Logger::Base>&);

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

  Shift::Parameter::FactoryArray<Solver::Base> solvers_;
  Shift::Parameter::FactoryArray<Integrator::Base> integrators_;
  Shift::Parameter::FactoryArray<Logger::Base> loggers_;
  Units units_;

  FILE* debug_pointer_ = nullptr;
};

} // namespace Shift

#endif
