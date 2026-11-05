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
    DeclareParameters();
  }
  Directional(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Directional() override {}
  static std::unique_ptr<Directional> Create();
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
