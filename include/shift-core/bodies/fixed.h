#ifndef _SHIFT_FIXEDBODY_
#define _SHIFT_FIXEDBODY_

#include "shift-core/bodies/body.h"
#include "shift-core/factory.h"

namespace Shift {

namespace Joint {
class JointBase;
}

namespace Terrain {
class TerrainBase;
}

namespace Body {

class Fixed : public BodyBase {
public:
  Fixed()
    : BodyBase() {
    setName("fixed");
    addKeys();
  }
  Fixed(std::string name)
    : BodyBase(name) {
    addKeys();
  }
  ~Fixed() {}
  static Fixed* create();
  void appendTerrain(Terrain::TerrainBase*);
  void readJSON(Json::Value&) override;
  virtual void addKeys() override;
  static bool isRegistered() {
    return registered_;
  }

private:
  std::vector<Terrain::TerrainBase*> terrain_;
  static bool registered_;
};

} // namespace Body

} // namespace Shift

#endif
