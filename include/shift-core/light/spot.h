#ifndef SHIFT_LIGHT_SPOT_H_
#define SHIFT_LIGHT_SPOT_H_

#include "shift-core/factory.h"
#include "shift-core/light/base.h"

namespace Shift {

namespace Light {

class Spot : public Base {
public:
  Spot()
    : Base() {
    addKeys();
  }
  Spot(std::string name)
    : Base(name) {
    addKeys();
  }
  ~Spot() {}
  static std::unique_ptr<Spot> create();
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
