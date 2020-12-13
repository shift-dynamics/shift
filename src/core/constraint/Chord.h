#ifndef SHIFT_CONSTRAINT_CHORD_H_
#define SHIFT_CONSTRAINT_CHORD_H_

#include "shift/core/constraint/Base.h"
#include "shift/core/Factory.h"

#include <memory>

namespace shift {
namespace core {

namespace joint {
class JointBase;
}

namespace constraint {

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
  void setOriginalParentBody(shift::core::body::Base* body) {
    original_parent_body_pointer_ = body;
  }

  void setOriginalParentFrame(Frame* frame) {
    original_parent_frame_pointer_ = frame;
  }

private:
  static bool registered_;
  shift::core::body::Base* original_parent_body_pointer_ = nullptr;
  shift::core::Frame* original_parent_frame_pointer_ = nullptr;
};

} // namespace constraint
}
} // namespace shift

#endif
