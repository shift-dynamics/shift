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
  virtual ~Chord() override {}
  static std::unique_ptr<Chord> create();
  void readJSON(Json::Value&) override;
  virtual void addKeys();
  static bool isRegistered() {
    return registered_;
  }
  void setOriginalParentBody(Body::Base* body) {
    originalParentBodyPointer_ = body;
  }

  void setOriginalParentFrame(Frame* frame) {
    originalParentFramePointer_ = frame;
  }

private:
  static bool registered_;
  Shift::Body::Base* originalParentBodyPointer_ = NULL;
  Shift::Frame* originalParentFramePointer_ = NULL;
};

} // namespace Constraint

} // namespace Shift

#endif
