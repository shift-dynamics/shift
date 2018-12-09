#ifndef _SHIFT_DIRECTIONALLIGHT_H_
#define _SHIFT_DIRECTIONALLIGHT_H_

#include "shift-core/factory.h"
#include "shift-core/lights/light.h"

namespace Shift {

namespace Light {

class Directional : public LightBase {
public:
  Directional()
    : LightBase() {
    addKeys();
  }
  Directional(std::string name)
    : LightBase(name) {
    addKeys();
  }
  ~Directional() {}
  static Directional* create();
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
