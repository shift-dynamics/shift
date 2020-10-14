#ifndef SHIFT_FORCEMODULE_PID_H_
#define SHIFT_FORCEMODULE_PID_H_

#include <memory>

#include "shift/core/forcemodule/base.h"

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
    addParameters();
  }
  PID(std::string name)
    : Base(name) {
    equation_count_ = 1;
    addParameters();
  }
  virtual ~PID() override {}
  static std::unique_ptr<PID> create();
  void addParameters() override;
  virtual void connect() override;
  void setTarget(double);
  virtual void service(const double y[], double dy[]) override;

  static bool isRegistered() {
    return registered_;
  }

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
