#include "shift/core/forcemodule/pid.h"
#include "shift/core/context.h"
#include "shift/core/factory.h"

bool shift::ForceModule::PID::registered_ =
  shift::Factory<shift::ForceModule::Base>::get()->registerModule(
    "pid", shift::ForceModule::PID::create);

std::unique_ptr<shift::ForceModule::PID> shift::ForceModule::PID::create() {
  return std::make_unique<shift::ForceModule::PID>();
}

void shift::ForceModule::PID::addParameters() {
  shift::ForceModule::Base::addParameters();
  addParameter("kp", &kp_, "proportional gain", true);
  addParameter("ki", &ki_, "integral gain", true);
  addParameter("kd", &kd_, "derivative gain", true);
  addParameter("target", &target_, "target", false);
}

void shift::ForceModule::PID::setTarget(double target) {
  target_ = target;
}

void shift::ForceModule::PID::service(const double y[], double dy[]) {

  double current_value = 0;
  switch(target_type_) {
    case POSITION: {
      current_value = y[q_index_];
    } break;
    case VELOCITY: {
      current_value = y[context_->qCount() + dq_index_];
    } break;
  }

  double t;
  double delta_t;
  context_->currentTime(t, delta_t);

  const double* y_f = y + context_->qCount() + context_->dqCount() + y_index_;
  double* dy_f = dy + context_->qCount() + context_->dqCount() + y_index_;

  /* calculate current error */
  error_ = target_ - current_value;
  dy_f[0] = error_;

  /* calculate derivative of e using last time */
  double delta_error = error_ - last_error_;

  double de_dt = delta_error / std::max(1e-6, std::abs(delta_t));

  /* control effort */
  double u = kp_ * error_ + ki_ * y_f[0] + kd_ * de_dt;

  context_->jointWorkingForces()[dq_index_] += u;
}

void shift::ForceModule::PID::connect() {}
