#ifndef _SHIFT_CHORDCONSTRAINT_
#define _SHIFT_CHORDCONSTRAINT_

#include "shift-core/constraints/constraint.h"
#include "shift-core/factory.h"

#include <memory>

namespace Shift {

namespace Joint {
class JointBase;
}

namespace Constraint {

class Chord : public ConstraintBase {
public:
  Chord()
    : ConstraintBase() {
    addKeys();
  }
  Chord(std::string name)
    : ConstraintBase(name) {
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
