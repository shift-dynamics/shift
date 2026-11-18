#include "shift/core/controller/driver.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "ksl/matrix.h"
#include "ksl/screw.h"
#include "ksl/vec3.h"
#include "shift/core/assembly.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"

bool shift::core::controller::Driver::registered_ =
  shift::core::Factory<shift::core::controller::Base>::get()->RegisterModule(
    "driver", shift::core::controller::Driver::Create);

std::unique_ptr<shift::core::controller::Driver>
shift::core::controller::Driver::Create() {
  return std::make_unique<shift::core::controller::Driver>();
}

void shift::core::controller::Driver::DeclareParameters() {
  shift::core::controller::Base::DeclareParameters();
  DeclareParameter("body_frame_pair", &body_frame_pair_,
                   "the body of the vehicle, in the same assembly, and its "
                   "frame at the middle of the rear axle, x forward, z down",
                   true)
    .set_row_dim(2);
  DeclareParameter("path", &path_,
                   "the points of the path in the fixed frame, [x0, y0, x1, "
                   "y1, ...] (m), at least two",
                   true)
    .set_row_dim(-1);
  DeclareParameter("closed", &closed_,
                   "whether the path goes on from the last point to the "
                   "first, a loop; defaults to false, a path that ends at "
                   "rest",
                   false);
  DeclareParameter("wheelbase", &wheelbase_, "wheelbase of the vehicle (m)",
                   true);
  DeclareParameter("lookahead_distance", &lookahead_distance_,
                   "look-ahead at rest (m); defaults to 5", false);
  DeclareParameter("lookahead_time", &lookahead_time_,
                   "look-ahead added per m/s of speed (s); defaults to 0.5",
                   false);
  DeclareParameter("max_steering_angle", &max_steering_angle_,
                   "greatest steering angle of the front wheels (rad); "
                   "defaults to 0.6",
                   false);
  DeclareParameter("speed", &speed_,
                   "greatest target speed (m/s); defaults to 10", false)
    .set_runtime();
  DeclareParameter("lateral_acceleration", &lateral_acceleration_,
                   "greatest lateral acceleration in bends (m/s^2); defaults "
                   "to 4",
                   false);
  DeclareParameter("acceleration", &acceleration_,
                   "greatest acceleration of the speed profile (m/s^2); "
                   "defaults to 3",
                   false);
  DeclareParameter("deceleration", &deceleration_,
                   "greatest deceleration of the speed profile (m/s^2); "
                   "defaults to 5",
                   false);
  DeclareParameter("throttle_gain", &throttle_gain_,
                   "throttle per m/s below the target speed; defaults to 0.5",
                   false)
    .set_runtime();
  DeclareParameter("brake_gain", &brake_gain_,
                   "brake per m/s above the target speed; defaults to 0.3",
                   false)
    .set_runtime();
  DeclareParameter("shift_speeds", &shift_speeds_,
                   "speeds of the frame (m/s), increasing, above which the "
                   "gear is 2, 3, ...; optional, the gear is 1 if none",
                   false)
    .set_row_dim(-1);
  DeclareOutput("steering", &steering_,
                "steering angle of the front wheels, positive to the right "
                "(rad)");
  DeclareOutput("throttle", &throttle_, "throttle, 0 to 1");
  DeclareOutput("brake", &brake_, "brake, 0 to 1");
  DeclareOutput("gear", &gear_, "gear, from 1");
  DeclareOutput("target_speed", &target_speed_,
                "target speed at the goal point (m/s)");
  DeclareOutput("vehicle_speed", &measured_speed_,
                "speed of the frame along its x axis (m/s)");
  DeclareOutput("station", &station_,
                "distance along the path of the nearest point (m)");
  DeclareOutput("lateral_error", &lateral_error_,
                "distance of the frame to the right of the path (m)");
}

void shift::core::controller::Driver::Connect() {
  const std::string prefix = "Driver '" + get_name() + "': ";
  if(parent_assembly_ptr_ == nullptr) {
    throw shift::core::Exception("No parent assembly set for controller " +
                                 get_name());
  }
  try {
    body_ = (*parent_assembly_ptr_)["bodies"][body_frame_pair_[0]]
              .get_data<shift::core::body::Base>();
  } catch(...) {
    body_ = nullptr;
  }
  if(body_ == nullptr) {
    throw shift::core::Exception(prefix + "body '" + body_frame_pair_[0] +
                                 "' not found in its assembly");
  }
  frame_ = nullptr;
  for(auto& frame : body_->get_frames()) {
    if(frame->get_name() == body_frame_pair_[1]) {
      frame_ = frame.get();
    }
  }
  if(frame_ == nullptr) {
    throw shift::core::Exception(prefix + "frame '" + body_frame_pair_[1] +
                                 "' not found on body '" +
                                 body_frame_pair_[0] + "'");
  }
  anchor_ = nullptr;
  if(path_.size() % 2 != 0 || path_.size() < 4) {
    throw shift::core::Exception(prefix + "path must be pairs [x0, y0, x1, "
                                          "y1, ...], at least two points");
  }
  if(!(wheelbase_ > 0.0) || !(lookahead_distance_ > 0.0) ||
     !(lookahead_time_ >= 0.0) || !(max_steering_angle_ > 0.0) ||
     !(max_steering_angle_ < 0.5 * M_PI)) {
    throw shift::core::Exception(
      prefix + "wheelbase and lookahead_distance must be positive, "
               "lookahead_time not negative, and max_steering_angle between "
               "0 and pi/2");
  }
  if(!(speed_ >= 0.0) || !(lateral_acceleration_ > 0.0) ||
     !(acceleration_ > 0.0) || !(deceleration_ > 0.0) ||
     !(throttle_gain_ >= 0.0) || !(brake_gain_ >= 0.0)) {
    throw shift::core::Exception(
      prefix + "speed and the gains must not be negative, the "
               "accelerations must be positive");
  }
  for(size_t k = 1; k < shift_speeds_.size(); k++) {
    if(!(shift_speeds_[k] > shift_speeds_[k - 1])) {
      throw shift::core::Exception(prefix + "shift_speeds must increase");
    }
  }

  /* the points and their stations */
  x_.clear();
  y_.clear();
  for(size_t k = 0; k < path_.size(); k += 2) {
    x_.push_back(path_[k]);
    y_.push_back(path_[k + 1]);
  }
  if(closed_) {
    x_.push_back(x_.front());
    y_.push_back(y_.front());
  }
  stations_.assign(x_.size(), 0.0);
  for(size_t k = 1; k < x_.size(); k++) {
    const double length = std::hypot(x_[k] - x_[k - 1], y_[k] - y_[k - 1]);
    if(!(length > 0.0)) {
      throw shift::core::Exception(prefix + "two points of the path in a row "
                                            "are the same");
    }
    stations_[k] = stations_[k - 1] + length;
  }

  /* the speed of each point: the curvature of the circle through the path a
  wheelbase before and after it (Menger curvature, 4 area over the product of
  the sides) */
  const size_t n = x_.size();
  profile_.assign(n, speed_);
  for(size_t k = 0; k < n; k++) {
    double ax, ay, bx, by;
    PointAt(stations_[k] - wheelbase_, ax, ay);
    PointAt(stations_[k] + wheelbase_, bx, by);
    const double px = x_[k], py = y_[k];
    const double cross = (px - ax) * (by - ay) - (py - ay) * (bx - ax);
    const double sides = std::hypot(px - ax, py - ay) *
                         std::hypot(bx - px, by - py) *
                         std::hypot(bx - ax, by - ay);
    const double curvature = sides > 0.0 ? 2.0 * std::fabs(cross) / sides : 0.0;
    if(curvature > 0.0) {
      profile_[k] =
        std::min(profile_[k], std::sqrt(lateral_acceleration_ / curvature));
    }
  }
  if(!closed_) {
    profile_.back() = 0.0;
  }
  /* within the acceleration from each point to the next, and the
  deceleration; twice round a loop so that the passes carry over its ends */
  const int passes = closed_ ? 2 : 1;
  for(int pass = 0; pass < passes; pass++) {
    for(size_t k = n - 1; k-- > 0;) {
      const double ds = stations_[k + 1] - stations_[k];
      profile_[k] = std::min(
        profile_[k],
        std::sqrt(profile_[k + 1] * profile_[k + 1] + 2.0 * deceleration_ * ds));
    }
    if(closed_) {
      profile_.back() = std::min(profile_.back(), profile_.front());
      profile_.front() = profile_.back();
    }
  }
  for(int pass = 0; pass < passes; pass++) {
    for(size_t k = 1; k < n; k++) {
      const double ds = stations_[k] - stations_[k - 1];
      profile_[k] = std::min(
        profile_[k],
        std::sqrt(profile_[k - 1] * profile_[k - 1] + 2.0 * acceleration_ * ds));
    }
    if(closed_) {
      profile_.front() = std::min(profile_.front(), profile_.back());
      profile_.back() = profile_.front();
    }
  }
  /* an open path starts at rest: the profile rises from 0 at the first
  point, and the driver aims for it at the goal point, ahead */
  if(!closed_) {
    profile_.front() = 0.0;
    for(size_t k = 1; k < n; k++) {
      const double ds = stations_[k] - stations_[k - 1];
      profile_[k] = std::min(
        profile_[k],
        std::sqrt(profile_[k - 1] * profile_[k - 1] + 2.0 * acceleration_ * ds));
    }
  }
  /* the outputs before the first evaluation: at rest, braked */
  steering_ = throttle_ = 0.0;
  brake_ = 0.0;
  gear_ = 1.0;
}

double shift::core::controller::Driver::Wrap(double s) const {
  const double length = get_length();
  if(closed_ && length > 0.0) {
    s = std::fmod(s, length);
    if(s < 0.0) {
      s += length;
    }
  }
  return s;
}

void shift::core::controller::Driver::PointAt(double s, double& x,
                                              double& y) const {
  s = Wrap(s);
  const size_t n = stations_.size();
  size_t k = 0;
  if(s <= 0.0) {
    k = 0;
  } else if(s >= stations_.back()) {
    k = n - 2;
  } else {
    k = std::upper_bound(stations_.begin(), stations_.end(), s) -
        stations_.begin() - 1;
  }
  const double u =
    (s - stations_[k]) / (stations_[k + 1] - stations_[k]);
  x = x_[k] + u * (x_[k + 1] - x_[k]);
  y = y_[k] + u * (y_[k + 1] - y_[k]);
}

double shift::core::controller::Driver::ProfileSpeed(double s) const {
  s = Wrap(s);
  if(s <= 0.0) {
    return profile_.front();
  }
  if(s >= stations_.back()) {
    return profile_.back();
  }
  const size_t k = std::upper_bound(stations_.begin(), stations_.end(), s) -
                   stations_.begin() - 1;
  const double u = (s - stations_[k]) / (stations_[k + 1] - stations_[k]);
  return profile_[k] + u * (profile_[k + 1] - profile_[k]);
}

double shift::core::controller::Driver::Nearest(double x, double y,
                                                double& lateral) const {
  double best = INFINITY, station = 0.0;
  lateral = 0.0;
  for(size_t k = 0; k + 1 < x_.size(); k++) {
    const double tx = x_[k + 1] - x_[k], ty = y_[k + 1] - y_[k];
    const double length = stations_[k + 1] - stations_[k];
    const double u = std::clamp(
      ((x - x_[k]) * tx + (y - y_[k]) * ty) / (length * length), 0.0, 1.0);
    const double dx = x - (x_[k] + u * tx), dy = y - (y_[k] + u * ty);
    const double distance = dx * dx + dy * dy;
    if(distance < best) {
      best = distance;
      station = stations_[k] + u * length;
      /* the right of the tangent, with z down, is (-ty, tx) */
      lateral = (-dx * ty + dy * tx) / length;
    }
  }
  return station;
}

void shift::core::controller::Driver::Drive(const ksl_SE3_t& pose,
                                            double speed) {
  measured_speed_ = speed;
  double lateral = 0.0;
  station_ = Nearest(pose.t.x, pose.t.y, lateral);
  lateral_error_ = lateral;

  /* the goal point, and its coordinates along the frame's x and y axes */
  const double lookahead =
    lookahead_distance_ + lookahead_time_ * std::max(speed, 0.0);
  double gx, gy;
  PointAt(station_ + lookahead, gx, gy);
  const ksl_vec3_t goal = ksl_vec3(gx - pose.t.x, gy - pose.t.y, 0.0);
  const double ahead = ksl_dot_vv(&pose.R.v0, &goal);
  const double right = ksl_dot_vv(&pose.R.v1, &goal);
  const double distance = ahead * ahead + right * right;
  steering_ =
    distance > 0.0 ? std::atan(2.0 * wheelbase_ * right / distance) : 0.0;
  steering_ = std::clamp(steering_, -max_steering_angle_, max_steering_angle_);

  /* the speed of the profile at the goal point */
  const double end = closed_ ? INFINITY : get_length();
  target_speed_ = station_ + lookahead >= end
                    ? 0.0
                    : std::min(ProfileSpeed(station_ + lookahead), speed_);
  const double error = target_speed_ - speed;
  throttle_ = std::clamp(throttle_gain_ * error, 0.0, 1.0);
  brake_ = target_speed_ <= 0.0 ? 1.0
                                : std::clamp(-brake_gain_ * error, 0.0, 1.0);
  gear_ = 1.0;
  for(const double shift_speed : shift_speeds_) {
    if(speed > shift_speed) {
      gear_ += 1.0;
    }
  }
}

void shift::core::controller::Driver::FindAnchor() {
  /* the poses of the frames in the kinematic tree are computed with the
  motion: the frame's from that of one of them on the body */
  for(auto& frame : body_->get_frames()) {
    if(frame->is_in_tree() && (anchor_ == nullptr || frame.get() == frame_)) {
      anchor_ = frame.get();
    }
  }
  if(anchor_ == nullptr) {
    throw shift::core::Exception("Driver '" + get_name() + "': body '" +
                                 body_frame_pair_[0] +
                                 "' has no frame joined to another body");
  }
  ksl_product_dinvd(anchor_->get_d_wrt_origin(), frame_->get_d_wrt_origin(),
                    &offset_);
}

void shift::core::controller::Driver::Compute() {
  assert(context_);
  if(anchor_ == nullptr) {
    FindAnchor();
  }
  ksl_SE3_t d;
  ksl_product_dd(&context_->get_dhva().d[anchor_->get_index()], &offset_,
                 &d);
  /* the velocity of the frame origin t from the spatial velocity of the
  body, referred to the origin of the fixed frame: v + w x t */
  const ksl_screw_t v =
    context_->GetBodyVelocity(context_->GetBodyLoadIndex(body_));
  ksl_vec3_t w_t;
  ksl_cross_vv(&v.ang, &d.t, &w_t);
  const ksl_vec3_t velocity =
    ksl_vec3(v.lin.x + w_t.x, v.lin.y + w_t.y, v.lin.z + w_t.z);
  Drive(d, ksl_dot_vv(&d.R.v0, &velocity));
}

void shift::core::controller::Driver::UpdateOutputs(double, const double*) {
  Compute();
}

void shift::core::controller::Driver::Service(const double*, double*) {
  Compute();
}
