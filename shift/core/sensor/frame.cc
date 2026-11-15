#include "shift/core/sensor/frame.h"

#include "shift/core/assembly.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"
#include "ksl/vec3.h"
#include "ksl/screw.h"

bool shift::core::sensor::Frame::registered_ =
  shift::core::Factory<shift::core::sensor::Base>::get()->RegisterModule(
    "frame", shift::core::sensor::Frame::Create);

std::unique_ptr<shift::core::sensor::Frame>
shift::core::sensor::Frame::Create() {
  return std::make_unique<shift::core::sensor::Frame>();
}

void shift::core::sensor::Frame::DeclareParameters() {
  shift::core::sensor::Base::DeclareParameters();
  DeclareParameter("body_frame_pair", &body_frame_pair_,
                   "body and frame names, the body in the same assembly",
                   true)
    .set_row_dim(2);
  DeclareParameter("quantity", &quantity_,
                   "measured quantity: \"position\" or \"velocity\"", true);
  DeclareParameter("coordinate_system", &coordinate_system_,
                   "\"global\" (default) or \"local\" (axes of the frame)",
                   false);
  DeclareParameter("dof", &dof_, "component: Tx, Ty, Tz, Rx, Ry or Rz", true);
}

void shift::core::sensor::Frame::Connect() {
  const std::string prefix = "Sensor " + get_name() + ": ";
  if(parent_assembly_ptr_ == nullptr) {
    throw shift::core::Exception("No parent assembly set for sensor " +
                                 get_name());
  }
  if(body_frame_pair_.size() != 2) {
    throw shift::core::Exception(prefix +
                                 "body_frame_pair must name a body and a frame");
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

  static const char* const kDofs[6] = {"Tx", "Ty", "Tz", "Rx", "Ry", "Rz"};
  component_ = -1;
  for(int i = 0; i < 6; i++) {
    if(dof_ == kDofs[i]) {
      component_ = i;
    }
  }
  if(component_ < 0) {
    throw shift::core::Exception(prefix + "dof must be one of Tx, Ty, Tz, "
                                          "Rx, Ry, Rz, not \"" +
                                 dof_ + "\"");
  }
  if(quantity_ != "position" && quantity_ != "velocity") {
    throw shift::core::Exception(prefix + "quantity must be \"position\" or "
                                          "\"velocity\", not \"" +
                                 quantity_ + "\"");
  }
  if(coordinate_system_ != "global" && coordinate_system_ != "local") {
    throw shift::core::Exception(prefix + "coordinate_system must be "
                                          "\"global\" or \"local\"");
  }
  position_ = quantity_ == "position";
  local_ = coordinate_system_ == "local";
  if(position_ && (component_ > 2 || local_)) {
    throw shift::core::Exception(
      prefix + "positions are measured as Tx, Ty, Tz in global coordinates");
  }
  has_rate_ = position_;
  DeclareSignalOutputs();
}

void shift::core::sensor::Frame::Update(const double* y) {
  (void)y;
  assert(frame_ && context_);
  const ksl_SE3_t& d = context_->get_dhva().d[frame_->get_index()];

  /* the spatial velocity of the body is referred to the origin of the fixed
  frame; the velocity of the frame origin t is v + w x t */
  const ksl_screw_t v =
    context_->GetBodyVelocity(context_->GetBodyLoadIndex(body_));
  ksl_vec3_t velocity;
  ksl_cross_vv(&v.ang, &d.t, &velocity);
  ksl_vec3_t linear = {{v.lin.x + velocity.x, v.lin.y + velocity.y,
                        v.lin.z + velocity.z}};
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

  if(position_) {
    value_ = d.t.at[component_];
    rate_ = linear.at[component_];
  } else {
    value_ = component_ < 3 ? linear.at[component_]
                            : angular.at[component_ - 3];
  }
}
