#ifndef SHIFT_LIGHT_DIRECTIONAL_H_
#define SHIFT_LIGHT_DIRECTIONAL_H_

#include "shift/core/factory.h"
#include "shift/core/light/base.h"

namespace shift {
namespace core {
namespace light {

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
  static bool isRegistered() {
    return registered_;
  }

private:
  virtual void addParameters() override;
  static bool registered_;
};

} // namespace light
}
} // namespace shift

#endif
