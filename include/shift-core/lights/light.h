#ifndef _SHIFT_LIGHT_H_
#define _SHIFT_LIGHT_H_

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

enum class LightType { POINT, SPOT, DIRECTIONAL };
/*!
@brief Shift Frame class
*/
class Light
  : public Base
  , public Serialize {

public:
  Light(std::string name)
    : Base(name) {
    addSerialEntry("name", SerialType::STRING, &name_);
    addSerialEntry("active", SerialType::BOOLEAN, &active_);
    addSerialEntry("definition", SerialType::STRING, &definition_);
    addSerialEntry("parent", SerialType::STRING_ARRAY, &parent_);
    addSerialEntry("Ka", SerialType::DOUBLE_ARRAY, &Ka_, 3);
    addSerialEntry("Kd", SerialType::DOUBLE_ARRAY, &Kd_, 3);
    addSerialEntry("Ks", SerialType::DOUBLE_ARRAY, &Ks_, 3);
  }
  ~Light() {}

private:
  std::string definition_;
  std::vector<std::string> parent_;

  ksl_vec3_t Ka_;
  ksl_vec3_t Kd_;
  ksl_vec3_t Ks_;
};

} // namespace Shift

#endif
