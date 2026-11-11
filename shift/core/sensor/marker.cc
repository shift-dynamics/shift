#include "shift/core/sensor/marker.h"

#include <cassert>
#include <cmath>

#include "ksl/matrix.h"
#include "ksl/quaternion.h"
#include "ksl/screw.h"
#include "ksl/vec3.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"

bool shift::core::sensor::Marker::registered_ =
  shift::core::Factory<shift::core::sensor::Base>::get()->RegisterModule(
    "marker", shift::core::sensor::Marker::Create);

std::unique_ptr<shift::core::sensor::Marker>
shift::core::sensor::Marker::Create() {
  return std::make_unique<shift::core::sensor::Marker>();
}

void shift::core::sensor::Marker::DeclareParameters() {
  shift::core::sensor::Base::DeclareParameters();
  DeclareParameter("frames", &frame_names_,
                   "the frames reported, as [body, frame] pairs; a body is "
                   "found by name anywhere in the model (its fully "
                   "qualified name if the name is not unique)",
                   true)
    .set_row_dim(-1)
    .set_column_dim(2);
  DeclareParameter("coordinate_system", &coordinate_system_,
                   "axes of the velocities: \"global\" (default) or "
                   "\"local\" (axes of each frame)",
                   false);
}

void shift::core::sensor::Marker::Connect() {
  const std::string prefix = "Sensor " + get_name() + ": ";
  if(coordinate_system_ != "global" && coordinate_system_ != "local") {
    throw shift::core::Exception(prefix + "coordinate_system must be "
                                          "\"global\" or \"local\"");
  }
  local_ = coordinate_system_ == "local";
  bodies_.clear();
  frames_.clear();
  for(const auto& pair : frame_names_) {
    if(pair.size() != 2) {
      throw shift::core::Exception(prefix + "each of frames must name a body "
                                            "and a frame");
    }
    shift::core::body::Base* body =
      context_->FindActiveBody(pair[0], get_parent_assembly());
    if(body == nullptr) {
      throw shift::core::Exception(prefix + "body '" + pair[0] +
                                   "' not found among the active bodies");
    }
    shift::core::Frame* found = nullptr;
    for(auto& frame : body->get_frames()) {
      if(frame->get_name() == pair[1]) {
        found = frame.get();
      }
    }
    if(found == nullptr) {
      throw shift::core::Exception(prefix + "frame '" + pair[1] +
                                   "' not found on body '" + pair[0] + "'");
    }
    bodies_.push_back(body);
    frames_.push_back(found);
  }
  values_.assign(kValuesPerFrame * frames_.size(), 0.0);
}

std::vector<std::string>
shift::core::sensor::Marker::get_output_names() const {
  static const char* const kNames[kValuesPerFrame] = {
    "_x",  "_y",  "_z",  "_qw", "_qx", "_qy", "_qz", "_roll",
    "_pitch", "_yaw", "_vx", "_vy", "_vz", "_wx", "_wy", "_wz"};
  std::vector<std::string> names;
  for(const auto& pair : frame_names_) {
    for(const char* component : kNames) {
      names.push_back(pair[0] + "_" + pair[1] + component);
    }
  }
  return names;
}

void shift::core::sensor::Marker::UpdateOutputs(double, const double*) {
  assert(context_);
  for(size_t k = 0; k < frames_.size(); k++) {
    /* frames that no joint or module uses have no place in the
    topology: compose the pose of the body's origin with the frame's */
    ksl_SE3_t d = *frames_[k]->get_d_wrt_origin();
    if(bodies_[k] != context_->get_fixed_body()) {
      const ksl_SE3_t origin =
        context_->get_body_origin_pose(bodies_[k], context_->get_dhva());
      ksl_product_dd(&origin, frames_[k]->get_d_wrt_origin(), &d);
    }
    double* out = values_.data() + kValuesPerFrame * k;
    out[0] = d.t.x;
    out[1] = d.t.y;
    out[2] = d.t.z;

    ksl_quaternion_t q;
    ksl_mat3x3_toQuaternion(&d.R, &q);
    /* the sign of a quaternion is arbitrary; take w >= 0 so that the
    logged components are continuous for rotations under half a turn */
    const double sign = q.w < 0.0 ? -1.0 : 1.0;
    out[3] = sign * q.w;
    out[4] = sign * q.x;
    out[5] = sign * q.y;
    out[6] = sign * q.z;

    /* R = Rz(yaw) Ry(pitch) Rx(roll); the columns v0, v1, v2 of R are the
    frame's axes: R(2,0) = -sin(pitch), R(2,1) = cos(pitch) sin(roll),
    R(2,2) = cos(pitch) cos(roll), R(1,0) = sin(yaw) cos(pitch), R(0,0) =
    cos(yaw) cos(pitch) */
    out[7] = std::atan2(d.R.v1.z, d.R.v2.z);
    out[8] = std::asin(std::fmax(-1.0, std::fmin(1.0, -d.R.v0.z)));
    out[9] = std::atan2(d.R.v0.y, d.R.v0.x);

    /* the spatial velocity of the body is referred to the origin of the
    fixed frame; the velocity of the frame origin t is v + w x t */
    const ksl_screw_t v =
      context_->GetBodyVelocity(context_->GetBodyLoadIndex(bodies_[k]));
    ksl_vec3_t w_x_t;
    ksl_cross_vv(&v.ang, &d.t, &w_x_t);
    ksl_vec3_t linear = {
      {v.lin.x + w_x_t.x, v.lin.y + w_x_t.y, v.lin.z + w_x_t.z}};
    ksl_vec3_t angular = v.ang;
    if(local_) {
      /* components along the axes of the frame: R^T v */
      const ksl_vec3_t* axes[3] = {&d.R.v0, &d.R.v1, &d.R.v2};
      ksl_vec3_t l, a;
      for(int i = 0; i < 3; i++) {
        l.at[i] = ksl_dot_vv(axes[i], &linear);
        a.at[i] = ksl_dot_vv(axes[i], &angular);
      }
      linear = l;
      angular = a;
    }
    for(int i = 0; i < 3; i++) {
      out[10 + i] = linear.at[i];
      out[13 + i] = angular.at[i];
    }
  }
}

void shift::core::sensor::Marker::AppendOutputs(
  std::vector<double>& values) const {
  values.insert(values.end(), values_.begin(), values_.end());
}
