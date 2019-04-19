#ifndef SHIFT_MODELPARAMETERS_H_
#define SHIFT_MODELPARAMETERS_H_

#include <json/json.h>
#include <string>
#include <vector>

#include "ksl/vec3.h"

#include "shift-core/base.h"
#include "shift-core/integrator/base.h"
#include "shift-core/logger/base.h"
#include "shift-core/serialize.h"
#include "shift-core/solver/base.h"
#include "shift-core/units.h"

namespace Shift {

class Context;
/*!
\brief Shift Assembly datastructure
*/
class ModelParameters
  : public Shift::Base
  , public Serialize {

public:
  ModelParameters(Shift::Context* context)
    : Shift::Base("") {
    context_ = context;
    gravity_ = ksl_vec3(0.0, 0.0, 0.0);
    addKeys();
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
    return epsilonRankDetermination_;
  }

  double epsilsonNewtonRaphson() const {
    return epsilsonNewtonRaphson_;
  }

  /*! \brief get infinity norm multiplier -- used as a heuristic for when to
   * reapply gaussian elimination with complete row and column pivoting */
  double infinityNormMultiplier() const {
    return infinityNormMuliplier_;
  }

  void enableContactDetection(bool);

  void readJSON(Json::Value&);

  void writeJSON(Json::Value&);

  void print();

  void appendSolver(std::unique_ptr<Solver::Base>&);
  void appendIntegrator(std::unique_ptr<Integrator::Base>&);
  void appendLogger(std::unique_ptr<Logger::Base>&);

  FILE* debugFilePointer() const {
    return debugFilePointer_;
  }

private:
  void addKeys();

  std::string version_;
  std::vector<std::string> includeDirectories_;
  ksl_vec3_t gravity_;
  double epsilsonNewtonRaphson_ = 1e-7;
  double epsilonRankDetermination_ = 1e-9;
  double infinityNormMuliplier_ = 2.0;

  bool contactDetectionEnabled_ = false;
  ksl_vec3_t viewportBackgroundColor_ = {{1.0, 1.0, 1.0}};

  std::vector<std::unique_ptr<Solver::Base>> solvers_;
  std::vector<std::unique_ptr<Integrator::Base>> integrators_;
  std::vector<std::unique_ptr<Logger::Base>> loggers_;
  Units units_;
  Shift::Context* context_ = NULL;
  FILE* debugFilePointer_ = NULL;
};

} // namespace Shift

#endif
