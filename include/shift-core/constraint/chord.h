#ifndef SHIFT_CONSTRAINT_CHORD_H_
#define SHIFT_CONSTRAINT_CHORD_H_

#include "shift-core/constraint/base.h"
#include "shift-core/factory.h"

#include <memory>

namespace Shift {

namespace Joint {
class JointBase;
}

namespace Constraint {

class Chord : public Base {
public:
  Chord()
    : Base() {
    addKeys();
  }
  Chord(std::string name)
    : Base(name) {
    addKeys();
  }
  ~Chord() {}
  static std::unique_ptr<Chord> create();
  void readJSON(Json::Value&) override;
  virtual void addKeys();
  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
};

} // namespace Constraint

} // namespace Shift

#endif
