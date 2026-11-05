#ifndef SHIFT_CONSTRAINT_CHORD_H_
#define SHIFT_CONSTRAINT_CHORD_H_

#include "shift/core/constraint/base.h"
#include "shift/core/factory.h"

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
    DeclareParameters();
  }
  Chord(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Chord() override {}
  static std::unique_ptr<Chord> Create();
  static bool is_registered() {
    return registered_;
  }
  void set_original_parent_body(shift::core::body::Base* body) {
    original_parent_body_pointer_ = body;
  }

  void set_original_parent_frame(Frame* frame) {
    original_parent_frame_pointer_ = frame;
  }

  virtual void DeclareParameters() override;

private:
  static bool registered_;
  shift::core::body::Base* original_parent_body_pointer_ = nullptr;
  shift::core::Frame* original_parent_frame_pointer_ = nullptr;
};

} // namespace constraint
}
} // namespace shift

#endif
