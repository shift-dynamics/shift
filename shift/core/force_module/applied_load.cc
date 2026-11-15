#include "shift/core/force_module/applied_load.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include "ksl/mat3x3.h"

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"

bool shift::core::force_module::AppliedLoad::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "applied_load", shift::core::force_module::AppliedLoad::Create);

std::unique_ptr<shift::core::force_module::AppliedLoad>
shift::core::force_module::AppliedLoad::Create() {
  return std::make_unique<shift::core::force_module::AppliedLoad>();
}

void shift::core::force_module::AppliedLoad::DeclareParameters() {
  DeclareParameter("body", &body_name_, "the body the load acts on", true);
  DeclareParameter("frame", &frame_name_,
                   "the frame of body at whose origin the load acts; "
                   "optional, default its origin",
                   false);
  DeclareParameter("reaction_body", &reaction_body_name_,
                   "takes the opposite load, at the same point; optional, "
                   "without it the load comes from outside the model",
                   false);
  DeclareParameter("coordinates", &coordinates_,
                   "\"frame\" (default), turning with the body, or \"fixed\"",
                   false);
  DeclareParameter("force", &force_, "optional, default zero", false)
    .set_runtime();
  DeclareParameter("moment", &moment_, "optional, default zero", false)
    .set_runtime();
  DeclareParameter("scale", &scale_,
                   "multiplies the load, for a schedule to change it; "
                   "optional, default 1",
                   false)
    .set_runtime();
  DeclareParameter("scale_curve", &scale_curve_values_,
                   "pairs [t0, s0, t1, s1, ...] of time and scale, in "
                   "increasing time; optional",
                   false)
    .set_row_dim(-1);
  DeclareParameter("scale_file", &scale_file_,
                   "text file of the same pairs, separated by blanks, commas "
                   "or line breaks; in place of scale_curve",
                   false);
  DeclareParameter("pulse_start", &pulse_start_,
                   "the time the pulse starts; optional, default 0", false);
  DeclareParameter("pulse_duration", &pulse_duration_,
                   "makes the load an impulse of this length, whose integral "
                   "over time is force times the duration; optional",
                   false);
  DeclareOutput("load_scale", &output_scale_,
                "the scale of the load at the time, the product of scale and "
                "of the pulse or the curve");
}

void shift::core::force_module::AppliedLoad::Connect() {
  const std::string prefix = "Applied load " + get_name() + ": ";
  body_ = context_->FindActiveBody(body_name_, get_parent_assembly());
  if(!body_) {
    throw shift::core::Exception(prefix + "body " + body_name_ +
                                 " not found among the active bodies");
  }
  frame_ = nullptr;
  for(auto& frame : body_->get_frames()) {
    if(frame->get_name() == frame_name_ ||
       (frame_name_ == "origin" && frame->is_origin())) {
      frame_ = frame.get();
    }
  }
  if(!frame_) {
    throw shift::core::Exception(prefix + "frame " + frame_name_ +
                                 " not found on body " + body_name_);
  }
  reaction_body_ = nullptr;
  if(!reaction_body_name_.empty()) {
    reaction_body_ =
      context_->FindActiveBody(reaction_body_name_, get_parent_assembly());
    if(!reaction_body_) {
      throw shift::core::Exception(prefix + "body " + reaction_body_name_ +
                                   " not found among the active bodies");
    }
  }
}

void shift::core::force_module::AppliedLoad::Load() {
  const std::string prefix = "Applied load " + get_name();
  if(coordinates_ != "frame" && coordinates_ != "fixed") {
    throw shift::core::Exception(prefix + ": coordinates must be \"frame\" or "
                                          "\"fixed\", not \"" + coordinates_ +
                                 "\"");
  }
  if(pulse_duration_ < 0.0) {
    throw shift::core::Exception(prefix + ": pulse_duration must not be "
                                          "negative");
  }
  if(!scale_file_.empty()) {
    if(!scale_curve_values_.empty()) {
      throw shift::core::Exception(prefix + ": give scale_curve or "
                                            "scale_file, not both");
    }
    const std::string path = context_->FindFile(scale_file_);
    std::ifstream in(path);
    if(path.empty() || !in) {
      throw shift::core::Exception(prefix + ": could not open " + scale_file_);
    }
    std::stringstream text;
    text << in.rdbuf();
    std::string s = text.str();
    std::replace(s.begin(), s.end(), ',', ' ');
    std::istringstream numbers(s);
    double value;
    while(numbers >> value) {
      scale_curve_values_.push_back(value);
    }
    if(!numbers.eof()) {
      throw shift::core::Exception(prefix + ": " + path + " holds a value "
                                                          "that is not a "
                                                          "number");
    }
  }
  if(!scale_curve_values_.empty()) {
    curve_ = Table::FromPairs(scale_curve_values_,
                              Table::Extrapolation::kHold,
                              prefix + ": scale_curve");
  }
  load_index_ = reaction_index_ = -2;
}

double shift::core::force_module::AppliedLoad::Scale(double t) const {
  double s = scale_;
  if(!curve_.empty()) {
    s *= curve_.Evaluate(t);
  }
  if(pulse_duration_ > 0.0) {
    const double u = (t - pulse_start_) / pulse_duration_;
    s *= u > 0.0 && u < 1.0 ? 1.0 - std::cos(2.0 * M_PI * u) : 0.0;
  }
  return s;
}

void shift::core::force_module::AppliedLoad::Service(const double*, double*) {
  if(load_index_ == -2) {
    load_index_ = context_->GetBodyLoadIndex(body_);
    if(reaction_body_) {
      reaction_index_ = context_->GetBodyLoadIndex(reaction_body_);
    }
  }
  const double s = Scale(context_->get_force_time());
  if(s == 0.0) {
    return;
  }
  ksl_SE3_t pose = *frame_->get_d_wrt_origin();
  if(body_ != context_->get_fixed_body()) {
    const ksl_SE3_t origin =
      context_->get_body_origin_pose(body_, context_->get_dhva());
    ksl_product_dd(&origin, frame_->get_d_wrt_origin(), &pose);
  }
  ksl_coscrew_t load;
  if(coordinates_ == "frame") {
    ksl_product_drv(&pose.R, &force_, &load.lin);
    ksl_product_drv(&pose.R, &moment_, &load.ang);
  } else {
    load.lin = force_;
    load.ang = moment_;
  }
  for(int c = 0; c < 6; c++) {
    load.at[c] *= s;
  }
  /* as a spatial force in the fixed frame referred to its origin */
  ksl_vec3_t p_cross_f;
  ksl_cross_vv(&pose.t, &load.lin, &p_cross_f);
  for(int c = 0; c < 3; c++) {
    load.ang.at[c] += p_cross_f.at[c];
  }
  context_->AddBodyLoad(load_index_, load);
  if(reaction_body_) {
    for(int c = 0; c < 6; c++) {
      load.at[c] = -load.at[c];
    }
    context_->AddBodyLoad(reaction_index_, load);
  }
}

void shift::core::force_module::AppliedLoad::UpdateOutputs(double t,
                                                           const double*) {
  output_scale_ = Scale(t);
}
