#ifndef SHIFT_LIGHT_DIRECTIONAL_H_
#define SHIFT_LIGHT_DIRECTIONAL_H_

#include "shift-core/factory.h"
#include "shift-core/light/base.h"

namespace Shift {

namespace Light {

class Directional : public Base {
public:
  Directional()
    : Base() {
    addKeys();
  }
  Directional(std::string name)
    : Base(name) {
    addKeys();
  }
  ~Directional() {}
  static std::unique_ptr<Directional> create();
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
