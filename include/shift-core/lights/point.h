#ifndef _SHIFT_POINTLIGHT_H_
#define _SHIFT_POINTLIGHT_H_

#include "shift-core/factory.h"
#include "shift-core/lights/light.h"

namespace Shift {

namespace Light {

class Point : public LightBase {
public:
  Point()
    : LightBase() {
    addKeys();
  }
  Point(std::string name)
    : LightBase(name) {
    addKeys();
  }
  ~Point() {}
  static Point* create();
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
