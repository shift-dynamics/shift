#ifndef SHIFT_BODY_FIXED_H_
#define SHIFT_BODY_FIXED_H_

#include "shift-core/body/base.h"
#include "shift-core/factory.h"

#include <memory>

namespace Shift {

namespace Joint {
class Base;
}

namespace Terrain {
class Base;
}

namespace Body {

class Fixed : public Base {
public:
  Fixed()
    : Base() {
    // setName("fixed");
    addKeys();
  }
  Fixed(std::string name)
    : Base(name) {
    addKeys();
  }
  ~Fixed() {}
  static std::unique_ptr<Fixed> create();
  void appendTerrain(std::unique_ptr<Terrain::Base>&);
  void readJSON(Json::Value&) override;
  virtual void addKeys() override;
  static bool isRegistered() {
    return registered_;
  }

private:
  std::vector<std::unique_ptr<Terrain::Base>> terrain_;
  static bool registered_;
};

} // namespace Body

} // namespace Shift

#endif
