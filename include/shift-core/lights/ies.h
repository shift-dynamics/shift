#ifndef _SHIFT_IESLIGHT_H_
#define _SHIFT_IESLIGHT_H_

#include "shift-core/factory.h"
#include "shift-core/lights/light.h"

namespace Shift {

namespace Light {

class IES : public LightBase {
public:
  IES()
    : LightBase() {
    addKeys();
  }
  IES(std::string name)
    : LightBase(name) {
    addKeys();
  }
  ~IES() {}
  static std::unique_ptr<IES> create();
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
