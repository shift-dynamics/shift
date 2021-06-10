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
    DeclareParameters();
  }
  IES(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~IES() override {}
  static std::unique_ptr<IES> Create();
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
