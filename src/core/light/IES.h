#ifndef SHIFT_LIGHT_IES_H_
#define SHIFT_LIGHT_IES_H_

#include "shift/core/Factory.h"
#include "shift/core/light/Base.h"

namespace shift {
namespace core {
namespace light {

class IES : public Base {
public:
  IES()
    : Base() {
    addParameters();
  }
  IES(std::string name)
    : Base(name) {
    addParameters();
  }
  virtual ~IES() override {}
  static std::unique_ptr<IES> create();
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
