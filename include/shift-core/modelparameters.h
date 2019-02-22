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

/*!
\brief Shift Assembly datastructure
*/
class ModelParameters
  : public Shift::Base
  , public Serialize {

public:
  ModelParameters(std::string name)
    : Shift::Base(name) {
    gravity_ = ksl_vec3(0.0, 0.0, 0.0);
    addKeys();
  }
  ModelParameters()
    : Shift::Base("") {
    gravity_ = ksl_vec3(0.0, 0.0, 0.0);
    addKeys();
  }
  ~ModelParameters() {}

  /*! \brief retrieve gravity vector */
  const ksl_vec3_t* getGravity() {
    return &gravity_;
  }

  /*! \brief set gravity vector */
  void setGravity(ksl_vec3_t);

  /*! \brief set gravity vector */
  void setGravity(double, double, double);

  void enableContactDetection(bool);

  void readJSON(Json::Value&);

  void writeJSON(Json::Value&);

  void print();

  void appendSolver(std::unique_ptr<Solver::Base>&);
  void appendIntegrator(std::unique_ptr<Integrator::Base>&);
  void appendLogger(std::unique_ptr<Logger::Base>&);

private:
  void addKeys();

  std::string version_;
  std::vector<std::string> include_directories_;
  ksl_vec3_t gravity_;

  bool contact_detection_enabled_ = false;
  ksl_vec3_t viewport_background_color_ = {{1.0, 1.0, 1.0}};

  std::vector<std::unique_ptr<Solver::Base>> solvers_;
  std::vector<std::unique_ptr<Integrator::Base>> integrators_;
  std::vector<std::unique_ptr<Logger::Base>> loggers_;
  Units units_;
};

} // namespace Shift

#endif
