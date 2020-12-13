#ifndef SHIFT_LIGHT_SPOT_H_
#define SHIFT_LIGHT_SPOT_H_

#include "shift/core/Factory.h"
#include "shift/core/light/Base.h"

namespace shift {
namespace core {
namespace light {

class Spot : public Base {
public:
  Spot()
    : Base() {
    addParameters();
  }
  Spot(std::string name)
    : Base(name) {
    addParameters();
  }
  virtual ~Spot() override {}
  static std::unique_ptr<Spot> create();
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
