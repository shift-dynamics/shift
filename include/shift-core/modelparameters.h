#ifndef _SHIFT_MODELPARAMETERS_H_
#define _SHIFT_MODELPARAMETERS_H_

#include <string>
#include <vector>

#include "ksl/vector.h"

#include "shift-core/base.h"

namespace Shift {

/*!
\brief Shift Assembly datastructure
*/
class ModelParameters : Base {

public:
  ModelParameters(std::string name)
    : Base(name) {
    gravity_ = ksl_vec3(0.0, 0.0, 0.0);
  }
  ~ModelParameters() {}

  /*! \brief retrieve gravity vector */
  const ksl_vec3_t& getGravity();

  /*! \brief retrieve angle units */
  const std::string& getUnitsAngle();

  /*! \brief retrieve length units */
  const std::string& getUnitsLength();

  /*! \brief retrieve mass units */
  const std::string& getUnitsMass();

  /*! \brief retrieve time units */
  const std::string& getUnitsTime();

  /*! \brief set gravity vector */
  void setGravity(ksl_vec3_t);

  /*! \brief set gravity vector */
  void setGravity(double, double, double);

  /*! \brief set angle units */
  void setUnitsAngle(const std::string);

  /*! \brief set length units */
  void setUnitsLength(const std::string);

  /*! \brief set mass units */
  void setUnitsMass(const std::string);

  /*! \brief set time units */
  void setUnitsTime(const std::string);

  void enableContactDetection(bool);

private:
  std::string version_;
  std::vector<std::string> include_directories_;
  ksl_vec3_t gravity_;

  struct units {
    std::string angle = "radians";
    std::string length = "meters";
    std::string mass = "kilograms";
    std::string time = "seconds";
  } units_;

  bool contact_detection_enabled_;

  double time_span_;
  double time_step_;

  std::string log_file_;
  std::string debug_file_;

  double pivot_ratio_multiple_;
};

} // namespace Shift

#endif
