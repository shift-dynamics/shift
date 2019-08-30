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
    addParameters();
  }
  Directional(std::string name)
    : Base(name) {
    addParameters();
  }
  virtual ~Directional() override {}
  static std::unique_ptr<Directional> create();
  virtual void addParameters() override;
  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
};

} // namespace Light

} // namespace Shift

#endif
