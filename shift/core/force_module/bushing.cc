#include "shift/core/force_module/bushing.h"

#include <algorithm>
#include <fstream>
#include <sstream>

#include "ksl/mat3x3.h"
#include "ksl/matrix.h"
#include "ksl/vec3.h"

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"
#include "shift/core/utility.h"

bool shift::core::force_module::Bushing::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "bushing", shift::core::force_module::Bushing::Create);

std::unique_ptr<shift::core::force_module::Bushing>
shift::core::force_module::Bushing::Create() {
  return std::make_unique<shift::core::force_module::Bushing>();
}

void shift::core::force_module::Bushing::DeclareParameters() {
  DeclareParameter("frames", &frame_names_,
                   "[body, frame] of each frame joined, two or more; each "
                   "pair of frames is joined",
                   true)
    .set_row_dim(-1)
    .set_column_dim(2);
  DeclareParameter("stiffness", &stiffness_values_,
                   "6 m numbers (diagonal) or (6 m)^2 (full matrix, row by "
                   "row) for m pairs of frames; optional, default zero",
                   false)
    .set_row_dim(-1);
  DeclareParameter("damping", &damping_values_,
                   "6 m numbers (diagonal) or (6 m)^2 (full matrix, row by "
                   "row); optional, default zero",
                   false)
    .set_row_dim(-1);
  DeclareParameter("stiffness_file", &stiffness_file_,
                   "text file of the full stiffness matrix, row by row; in "
                   "place of stiffness",
                   false);
  DeclareParameter("damping_file", &damping_file_,
                   "text file of the full damping matrix, row by row; in "
                   "place of damping",
                   false);
  DeclareParameter("preload", &preload_,
                   "force and moment of each pair at rest, 6 m numbers; "
                   "optional, default zero",
                   false)
    .set_row_dim(-1);
  DeclareParameter("rest_pose", &rest_pose_,
                   "\"initial\" (default): the relative poses as the model "
                   "starts; \"frames\": the frames of each pair coincide",
                   false);
}

std::string shift::core::force_module::Bushing::FindFile(
  const std::string& name) const {
  if(!name.empty() && name[0] == '/') {
    return name;
  }
  const std::string file =
    shift::core::utility::GetParentPath(context_->get_filename()) + "/" + name;
  if(shift::core::utility::FileExists(file)) {
    return file;
  }
  for(const auto& directory :
      (*context_)["model_parameters"]["include_directories"]
        .get<std::vector<std::string>>()) {
    const std::string candidate = directory + "/" + name;
    if(shift::core::utility::FileExists(candidate)) {
      return candidate;
    }
  }
  throw shift::core::Exception("Bushing " + get_name() + ": file " + name +
                               " not found");
}

void shift::core::force_module::Bushing::Connect() {
  const std::string prefix = "Bushing " + get_name() + ": ";
  if(frame_names_.size() < 2) {
    throw shift::core::Exception(prefix + "frames must name two or more "
                                 "frames");
  }
  attachments_.clear();
  for(const auto& names : frame_names_) {
    if(names.size() != 2) {
      throw shift::core::Exception(prefix + "each of frames must be "
                                   "[body, frame]");
    }
    Attachment a;
    a.body = context_->FindActiveBody(names[0]);
    if(!a.body) {
      throw shift::core::Exception(prefix + "body " + names[0] +
                                   " not found among the active bodies");
    }
    for(auto& frame : a.body->get_frames()) {
      if(frame->get_name() == names[1] ||
         (names[1] == "origin" && frame->is_origin())) {
        a.frame = frame.get();
      }
    }
    if(!a.frame) {
      throw shift::core::Exception(prefix + "frame " + names[1] +
                                   " not found on body " + names[0]);
    }
    attachments_.push_back(a);
  }
  pairs_.clear();
  for(size_t i = 0; i < attachments_.size(); i++) {
    for(size_t j = i + 1; j < attachments_.size(); j++) {
      Pair pair;
      pair.i = i;
      pair.j = j;
      pairs_.push_back(pair);
    }
  }
}

std::vector<double> shift::core::force_module::Bushing::Matrix(
  const std::string& key, const std::vector<double>& values,
  const std::string& file) const {
  const std::string prefix = "Bushing " + get_name() + ": ";
  const size_t n = 6 * pairs_.size();
  std::vector<double> matrix(n * n, 0.0);
  if(!values.empty() && !file.empty()) {
    throw shift::core::Exception(prefix + "give " + key + " or " + key +
                                 "_file, not both");
  }
  std::vector<double> full;
  if(!file.empty()) {
    const std::string path = FindFile(file);
    std::ifstream in(path);
    if(!in) {
      throw shift::core::Exception(prefix + "could not open " + path);
    }
    std::stringstream text;
    text << in.rdbuf();
    std::string s = text.str();
    std::replace(s.begin(), s.end(), ',', ' ');
    std::istringstream numbers(s);
    double value;
    while(numbers >> value) {
      full.push_back(value);
    }
    if(!numbers.eof()) {
      throw shift::core::Exception(prefix + path + " holds a value that is "
                                   "not a number");
    }
    if(full.size() != n * n) {
      throw shift::core::Exception(
        prefix + path + " holds " + std::to_string(full.size()) +
        " numbers; the full matrix of " + std::to_string(pairs_.size()) +
        " pairs has " + std::to_string(n * n));
    }
    return full;
  }
  if(values.empty()) {
    return matrix;
  }
  if(values.size() == n) {
    for(size_t k = 0; k < n; k++) {
      matrix[k * n + k] = values[k];
    }
    return matrix;
  }
  if(values.size() == n * n) {
    return values;
  }
  throw shift::core::Exception(
    prefix + key + " has " + std::to_string(values.size()) + " numbers; " +
    std::to_string(pairs_.size()) + " pairs need " + std::to_string(n) +
    " (diagonal) or " + std::to_string(n * n) + " (full matrix)");
}

ksl_SE3_t shift::core::force_module::Bushing::Pose(const Attachment& a) const {
  if(a.body == context_->get_fixed_body()) {
    return *a.frame->get_d_wrt_origin();
  }
  const ksl_SE3_t origin =
    context_->get_body_origin_pose(a.body, context_->get_dhva());
  ksl_SE3_t pose;
  ksl_product_dd(&origin, a.frame->get_d_wrt_origin(), &pose);
  return pose;
}

void shift::core::force_module::Bushing::Load() {
  const std::string prefix = "Bushing " + get_name() + ": ";
  const size_t n = 6 * pairs_.size();
  stiffness_ = Matrix("stiffness", stiffness_values_, stiffness_file_);
  damping_ = Matrix("damping", damping_values_, damping_file_);
  if(!preload_.empty() && preload_.size() != n) {
    throw shift::core::Exception(prefix + "preload needs " +
                                 std::to_string(n) + " numbers");
  }
  if(preload_.empty()) {
    preload_.assign(n, 0.0);
  }
  if(rest_pose_ != "initial" && rest_pose_ != "frames") {
    throw shift::core::Exception(prefix + "rest_pose must be \"initial\" or "
                                 "\"frames\", not \"" + rest_pose_ + "\"");
  }
  for(auto& pair : pairs_) {
    pair.rest = ksl_SE3_identity();
    if(rest_pose_ == "initial") {
      const ksl_SE3_t di = Pose(attachments_[pair.i]);
      const ksl_SE3_t dj = Pose(attachments_[pair.j]);
      ksl_product_dinvd(&di, &dj, &pair.rest);
    }
  }
  x_.assign(n, 0.0);
  v_.assign(n, 0.0);
  f_.assign(n, 0.0);
}

void shift::core::force_module::Bushing::Service(const double* y, double* dy) {
  Compute(y, dy);
  Apply();
}

void shift::core::force_module::Bushing::UpdateOutputs(double t,
                                                       const double* y) {
  (void)t;
  Compute(y, nullptr);
}

void shift::core::force_module::Bushing::Compute(const double* y, double* dy) {
  Measure();
  /* f = -K x - C v + preload */
  const size_t n = 6 * pairs_.size();
  for(size_t r = 0; r < n; r++) {
    double sum = preload_[r];
    const double* k_row = stiffness_.data() + r * n;
    const double* c_row = damping_.data() + r * n;
    for(size_t c = 0; c < n; c++) {
      sum -= k_row[c] * x_[c] + c_row[c] * v_[c];
    }
    f_[r] = sum;
  }
  AddForces(y, dy);
}

void shift::core::force_module::Bushing::Measure() {
  poses_.resize(attachments_.size());
  axes_.resize(pairs_.size());
  std::vector<ksl_screw_t> velocities(attachments_.size());
  for(size_t a = 0; a < attachments_.size(); a++) {
    Attachment& attachment = attachments_[a];
    if(attachment.load_index == -2) {
      attachment.load_index = context_->GetBodyLoadIndex(attachment.body);
    }
    poses_[a] = Pose(attachment);
    velocities[a] = context_->GetBodyVelocity(attachment.load_index);
  }

  /* the relative coordinates and rates of each pair, in its rest axes */
  for(size_t k = 0; k < pairs_.size(); k++) {
    const Pair& pair = pairs_[k];
    const ksl_SE3_t& di = poses_[pair.i];
    const ksl_SE3_t& dj = poses_[pair.j];
    ksl_SE3_t relative, deviation;
    ksl_product_dinvd(&di, &dj, &relative);
    ksl_product_dinvd(&pair.rest, &relative, &deviation);
    ksl_vec3_t axis;
    double angle;
    ksl_mat3x3_getAxisAngle(&deviation.R, &axis, &angle);
    ksl_product_drdr(&di.R, &pair.rest.R, &axes_[k]);
    for(int c = 0; c < 3; c++) {
      x_[6 * k + c] = deviation.t.at[c];
      x_[6 * k + 3 + c] = angle * axis.at[c];
    }
    /* velocities of the point at the origin of j as part of each body: the
    spatial velocities are referred to the fixed origin, v(p) = v + w x p */
    const ksl_screw_t& si = velocities[pair.i];
    const ksl_screw_t& sj = velocities[pair.j];
    ksl_vec3_t wi_p, wj_p;
    ksl_cross_vv(&si.ang, &dj.t, &wi_p);
    ksl_cross_vv(&sj.ang, &dj.t, &wj_p);
    ksl_vec3_t dv, dw;
    for(int c = 0; c < 3; c++) {
      dv.at[c] = (sj.lin.at[c] + wj_p.at[c]) - (si.lin.at[c] + wi_p.at[c]);
      dw.at[c] = sj.ang.at[c] - si.ang.at[c];
    }
    ksl_vec3_t dv_rest, dw_rest;
    ksl_product_drinvv(&axes_[k], &dv, &dv_rest);
    ksl_product_drinvv(&axes_[k], &dw, &dw_rest);
    for(int c = 0; c < 3; c++) {
      v_[6 * k + c] = dv_rest.at[c];
      v_[6 * k + 3 + c] = dw_rest.at[c];
    }
  }
}

void shift::core::force_module::Bushing::Apply() {
  /* the force and moment on j at its origin, and the opposite on i, as
  spatial forces in the fixed frame referred to its origin */
  for(size_t k = 0; k < pairs_.size(); k++) {
    const Pair& pair = pairs_[k];
    const ksl_vec3_t force_rest = {{f_[6 * k], f_[6 * k + 1], f_[6 * k + 2]}};
    const ksl_vec3_t moment_rest = {
      {f_[6 * k + 3], f_[6 * k + 4], f_[6 * k + 5]}};
    ksl_coscrew_t load;
    ksl_product_drv(&axes_[k], &force_rest, &load.lin);
    ksl_product_drv(&axes_[k], &moment_rest, &load.ang);
    ksl_vec3_t p_cross_f;
    ksl_cross_vv(&poses_[pair.j].t, &load.lin, &p_cross_f);
    for(int c = 0; c < 3; c++) {
      load.ang.at[c] += p_cross_f.at[c];
    }
    context_->AddBodyLoad(attachments_[pair.j].load_index, load);
    for(int c = 0; c < 6; c++) {
      load.at[c] = -load.at[c];
    }
    context_->AddBodyLoad(attachments_[pair.i].load_index, load);
  }
}

std::vector<std::string>
shift::core::force_module::Bushing::get_output_names() const {
  std::vector<std::string> names;
  for(size_t k = 0; k < pairs_.size(); k++) {
    for(const char* c : {"fx", "fy", "fz", "mx", "my", "mz"}) {
      names.push_back("pair_" + std::to_string(k) + "_" + c);
    }
  }
  return names;
}

void shift::core::force_module::Bushing::AppendOutputs(
  std::vector<double>& values) const {
  values.insert(values.end(), f_.begin(), f_.end());
}
