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
    addParameters();
  }
  Chord(std::string name)
    : Base(name) {
    addParameters();
  }
  virtual ~Chord() override {}
  static std::unique_ptr<Chord> create();
  virtual void addParameters() override;
  static bool isRegistered() {
    return registered_;
  }
  void setOriginalParentBody(Body::Base* body) {
    original_parent_body_pointer_ = body;
  }

  void setOriginalParentFrame(Frame* frame) {
    original_parent_frame_pointer_ = frame;
  }

private:
  static bool registered_;
  Shift::Body::Base* original_parent_body_pointer_ = nullptr;
  Shift::Frame* original_parent_frame_pointer_ = nullptr;
};

} // namespace Constraint

} // namespace Shift

#endif
