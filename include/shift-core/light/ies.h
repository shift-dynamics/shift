#ifndef SHIFT_LIGHT_IES_H_
#define SHIFT_LIGHT_IES_H_

#include "shift-core/factory.h"
#include "shift-core/light/base.h"

namespace Shift {

namespace Light {

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
