#ifndef SHIFT_LIGHT_POINT_H_
#define SHIFT_LIGHT_POINT_H_

#include "shift/core/Factory.h"
#include "shift/core/light/Base.h"

namespace shift {
namespace core {
namespace light {

class Point : public Base {
public:
  Point()
    : Base() {
    DeclareParameters();
  }
  Point(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Point() override {}
  static std::unique_ptr<Point> Create();
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
