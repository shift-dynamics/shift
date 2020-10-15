#ifndef SHIFT_LIGHT_POINT_H_
#define SHIFT_LIGHT_POINT_H_

#include "shift/core/factory.h"
#include "shift/core/light/base.h"

namespace shift {
namespace core {
namespace light {

class Point : public Base {
public:
  Point()
    : Base() {
    addParameters();
  }
  Point(std::string name)
    : Base(name) {
    addParameters();
  }
  virtual ~Point() override {}
  static std::unique_ptr<Point> create();
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
