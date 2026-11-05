#ifndef SHIFT_LIGHT_SPOT_H_
#define SHIFT_LIGHT_SPOT_H_

#include "shift/core/factory.h"
#include "shift/core/light/base.h"

namespace shift {
namespace core {
namespace light {

class Spot : public Base {
public:
  Spot()
    : Base() {
    DeclareParameters();
  }
  Spot(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Spot() override {}
  static std::unique_ptr<Spot> Create();
  static bool is_registered() {
    return registered_;
  }

private:
  virtual void DeclareParameters() override;
  static bool registered_;
};

} // namespace light
}
} // namespace shift

#endif
