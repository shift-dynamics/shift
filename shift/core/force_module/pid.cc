#include "shift/core/force_module/pid.h"
#include "shift/core/context.h"
#include "shift/core/factory.h"

bool shift::core::force_module::PID::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "pid", shift::core::force_module::PID::Create);

std::unique_ptr<shift::core::force_module::PID> shift::core::force_module::PID::Create() {
  return std::make_unique<shift::core::force_module::PID>();
}

void shift::core::force_module::PID::DeclareParameters() {
  shift::core::force_module::Base::DeclareParameters();
  DeclareParameter("kp", &kp_, "proportional gain", true);
  DeclareParameter("ki", &ki_, "integral gain", true);
  DeclareParameter("kd", &kd_, "derivative gain", true);
  DeclareParameter("target", &target_, "target", false);
}

void shift::core::force_module::PID::set_target(double target) {
  target_ = target;
}

void shift::core::force_module::PID::Service(const double y[], double dy[]) {

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

void shift::core::force_module::PID::Connect() {}
