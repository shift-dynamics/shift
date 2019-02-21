#ifndef SHIFT_LIGHT_POINT_H_
#define SHIFT_LIGHT_POINT_H_

#include "shift-core/factory.h"
#include "shift-core/light/base.h"

namespace Shift {

namespace Light {

class Point : public Base {
public:
  Point()
    : Base() {
    addKeys();
  }
  Point(std::string name)
    : Base(name) {
    addKeys();
  }
  virtual ~Point() override {}
  static std::unique_ptr<Point> create();
  void readJSON(Json::Value&) override;
  virtual void addKeys();
  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
};

} // namespace Light

} // namespace Shift

#endif
