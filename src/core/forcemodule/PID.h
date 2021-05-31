#ifndef SHIFT_FORCEMODULE_PID_H_
#define SHIFT_FORCEMODULE_PID_H_

#include <memory>

#include "shift/core/forcemodule/Base.h"

namespace shift {
namespace core {

namespace joint {
class JointBase;
}

namespace forcemodule {

class PID : public forcemodule::Base {
public:
  PID()
    : Base() {
    equation_count_ = 1;
    DeclareParameters();
  }
  PID(std::string name)
    : Base(name) {
    equation_count_ = 1;
    DeclareParameters();
  }
  virtual ~PID() override {}

  static std::unique_ptr<PID> Create();

  static bool is_registered() {
    return registered_;
  }

  void set_target(double);
  
  void DeclareParameters() override;
  virtual void Connect() override;
  virtual void Service(const double y[], double dy[]) override;

private:
  enum TargetType { POSITION = 0, VELOCITY } target_type_;
  static bool registered_;
  double last_error_ = 0.0;
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double target_ = 0.0;
  double error_ = 0.0;
  size_t q_index_ = 0;
  size_t dq_index_ = 0;
  size_t y_index_ = 0;
};

} // namespace forcemodule
}
} // namespace shift

#endif
