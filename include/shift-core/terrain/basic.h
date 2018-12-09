#ifndef _SHIFT_BASICTERRAIN_H_
#define _SHIFT_BASICTERRAIN_H_

#include "terrain.h"

namespace Shift {

namespace Terrain {

class Basic : public TerrainBase {
public:
  Basic()
    : TerrainBase() {
    addKeys();
  }
  Basic(std::string name)
    : TerrainBase(name) {
    addKeys();
  }
  ~Basic() {}
  static Basic* create();
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
