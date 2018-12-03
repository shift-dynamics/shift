#ifndef _SHIFT_BEHAVIOR_H_
#define _SHIFT_BEHAVIOR_H_

#include <functional>
#include <json/json.h>
#include <map>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

namespace Behavior {

/*!
@brief Abstract base class for Shift::Behavior C++ objects
*/
class BehaviorBase
  : public Base
  , public Serialize {

public:
  BehaviorBase(std::string name)
    : Base(name) {
    addKeys();
  }
  BehaviorBase()
    : Base("") {
    addKeys();
  }
  ~BehaviorBase() {}

  const std::string& getType() {
    return type_;
  }

  virtual void readJSON(Json::Value);

protected:
  void addKeys();

  std::string type_; //!< descriptive name of Behavior type
};

} // namespace Behavior

} // namespace Shift

#endif
