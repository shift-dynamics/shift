#ifndef _SHIFT_SPOTLIGHT_H_
#define _SHIFT_SPOTLIGHT_H_

#include "shift-core/factory.h"
#include "shift-core/lights/light.h"

namespace Shift {

namespace Light {

class Spot : public LightBase {
public:
  Spot()
    : LightBase() {
    addKeys();
  }
  Spot(std::string name)
    : LightBase(name) {
    addKeys();
  }
  ~Spot() {}
  static Spot* create();
  void readJSON(Json::Value&);
  virtual void addKeys();
  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
};

} // namespace Light

} // namespace Shift

#endif
