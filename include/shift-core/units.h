#ifndef SHIFT_UNITS_H_
#define SHIFT_UNITS_H_

#include <json/json.h>
#include <string>
#include <vector>

#include "ksl/vec3.h"

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

/*!
\brief Shift Assembly datastructure
*/
class Units
  : public Base
  , public Serialize {

public:
  Units(std::string name)
    : Base(name)
    , Serialize() {
    addKeys();
  }
  Units()
    : Base("")
    , Serialize() {
    addKeys();
  }
  ~Units() {}

  /*! \brief retrieve angle units */
  const std::string& getUnitsAngle();

  /*! \brief retrieve length units */
  const std::string& getUnitsLength();

  /*! \brief retrieve mass units */
  const std::string& getUnitsMass();

  /*! \brief retrieve time units */
  const std::string& getUnitsTime();

  /*! \brief set angle units */
  void setUnitsAngle(const std::string);

  /*! \brief set length units */
  void setUnitsLength(const std::string);

  /*! \brief set mass units */
  void setUnitsMass(const std::string);

  /*! \brief set time units */
  void setUnitsTime(const std::string);

  void print();

private:
  void addKeys();

  std::string angle_ = "radians";
  std::string length_ = "meters";
  std::string mass_ = "kilograms";
  std::string time_ = "seconds";
};

} // namespace Shift

#endif
