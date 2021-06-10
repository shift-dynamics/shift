#ifndef SHIFT_MODELPARAMETERS_H_
#define SHIFT_MODELPARAMETERS_H_

#include <string>
#include <vector>

#include "ksl/vec3.h"

#include "shift/core/Base.h"
#include "shift/core/integrator/Base.h"
#include "shift/core/logger/Base.h"
#include "shift/core/parameter/List.h"
#include "shift/core/solver/Base.h"
#include "shift/core/Units.h"

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
    DeclareParameters();
  }

  ModelParameters()
    : Base("") {
    DeclareParameters();
  }

  ~ModelParameters() {}

  FILE* get_debug_file_pointer() const {
    return debug_pointer_;
  }


  /*! \brief retrieve gravity vector */
  const ksl_vec3_t* get_gravity() const {
    return &gravity_;
  }

  /*! \brief get epsilon used for rank determination in Gaussian Elimination
   * with complete row and column pivoting */
  double get_epsilon_rank_determination() const {
    return epsilon_rank_determination_;
  }

  double get_epsilon_newton_raphson() const {
    return epsilson_newton_raphson_;
  }

  /*! \brief get infinity norm multiplier -- used as a heuristic for when to
   * reapply gaussian elimination with complete row and column pivoting */
  double get_infinity_norm_multiplier() const {
    return infinity_norm_muliplier_;
  }

  /*! \brief set gravity vector */
  void set_gravity(ksl_vec3_t);

  /*! \brief set gravity vector */
  void set_gravity(double, double, double);
  void enable_contact_detection(bool);

  void AppendSolver(std::unique_ptr<solver::Base>&);
  void AppendIntegrator(std::unique_ptr<integrator::Base>&);
  void AppendLogger(std::unique_ptr<logger::Base>&);

  /*! \brief allocate dynamic memory in solvers, integrators and loggers */
  void Initialize();


private:
  virtual void DeclareParameters() override;

  std::string version_;
  std::vector<std::string> include_directories_;
  ksl_vec3_t gravity_ = {{0.0, 0.0, 0.0}};
  double epsilson_newton_raphson_ = 1e-7;
  double epsilon_rank_determination_ = 1e-9;
  double infinity_norm_muliplier_ = 2.0;

  bool contact_detection_enabled_ = false;
  ksl_vec3_t viewport_background_color_ = {{1.0, 1.0, 1.0}};
  bool print_constraint_error_ = false;

  parameter::FactoryArray<solver::Base> solvers_;
  parameter::FactoryArray<integrator::Base> integrators_;
  parameter::FactoryArray<logger::Base> loggers_;
  Units units_;

  FILE* debug_pointer_ = nullptr;
};

} // namespace core
} // namespace shift

#endif
