#ifndef SHIFT_TERRAIN_BASIC_H_
#define SHIFT_TERRAIN_BASIC_H_

#include "shift-core/terrain/base.h"

namespace Shift {

namespace Terrain {

class Basic : public Base {
public:
  Basic()
    : Base() {
    addKeys();
  }
  Basic(std::string name)
    : Base(name) {
    addKeys();
  }
  ~Basic() {}
  static std::unique_ptr<Basic> create();
  void addKeys();
  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
};

} // namespace Terrain

} // namespace Shift

#endif
