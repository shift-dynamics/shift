#include "shift/core/forcemodule/PID.h"
#include "shift/core/Context.h"
#include "shift/core/Factory.h"

bool shift::core::forcemodule::PID::registered_ =
  shift::core::Factory<shift::core::forcemodule::Base>::get()->RegisterModule(
    "pid", shift::core::forcemodule::PID::Create);

std::unique_ptr<shift::core::forcemodule::PID> shift::core::forcemodule::PID::Create() {
  return std::make_unique<shift::core::forcemodule::PID>();
}

void shift::core::forcemodule::PID::DeclareParameters() {
  shift::core::forcemodule::Base::DeclareParameters();
  DeclareParameter("kp", &kp_, "proportional gain", true);
  DeclareParameter("ki", &ki_, "integral gain", true);
  DeclareParameter("kd", &kd_, "derivative gain", true);
  DeclareParameter("target", &target_, "target", false);
}

void shift::core::forcemodule::PID::set_target(double target) {
  target_ = target;
}

void shift::core::forcemodule::PID::Service(const double y[], double dy[]) {

  double current_value = 0;
  switch(target_type_) {
    case POSITION: {
      current_value = y[q_index_];
    } break;
    case VELOCITY: {
      current_value = y[context_->get_q_count() + dq_index_];
    } break;
  }

  double t;
  double delta_t;
  context_->get_current_time(t, delta_t);

  const double* y_f = y + context_->get_q_count() + context_->get_dq_count() + y_index_;
  double* dy_f = dy + context_->get_q_count() + context_->get_dq_count() + y_index_;

  /* calculate current error */
  error_ = target_ - current_value;
  dy_f[0] = error_;

  /* calculate derivative of e using last time */
  double delta_error = error_ - last_error_;

  double de_dt = delta_error / std::max(1e-6, std::abs(delta_t));

  /* control effort */
  double u = kp_ * error_ + ki_ * y_f[0] + kd_ * de_dt;

  context_->get_joint_working_forces()[dq_index_] += u;
}

void shift::core::forcemodule::PID::Connect() {}
