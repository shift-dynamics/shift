#include "shift/core/force_module/gear_mesh.h"

#include <algorithm>
#include <cmath>

#include "ksl/SE3.h"
#include "ksl/coscrew.h"
#include "ksl/screw.h"
#include "ksl/vec3.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"

bool shift::core::force_module::GearMesh::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "gear_mesh", shift::core::force_module::GearMesh::Create);

std::unique_ptr<shift::core::force_module::GearMesh>
shift::core::force_module::GearMesh::Create() {
  return std::make_unique<shift::core::force_module::GearMesh>();
}

void shift::core::force_module::GearMesh::DeclareParameters() {
  shift::core::force_module::Base::DeclareParameters();
  DeclareParameter("gear_1", &gear_1_names_,
                   "[body, frame] of gear 1; the frame is on its axis, with "
                   "the z axis along it",
                   true)
    .set_row_dim(2);
  DeclareParameter("gear_2", &gear_2_names_,
                   "[body, frame] of gear 2; its axis is parallel to that of "
                   "gear 1 and points the same way",
                   true)
    .set_row_dim(2);
  DeclareParameter("pitch_radius_1", &pitch_radius_1_,
                   "pitch radius of gear 1", true);
  DeclareParameter("pitch_radius_2", &pitch_radius_2_,
                   "pitch radius of gear 2", true);
  DeclareParameter("teeth_1", &teeth_1_, "number of teeth of gear 1", true);
  DeclareParameter("pressure_angle", &pressure_angle_,
                   "pressure angle of the teeth, radians; optional, default "
                   "20 degrees",
                   false);
  DeclareParameter("tooth_stiffness", &tooth_stiffness_,
                   "stiffness of one pair of teeth in contact, along the "
                   "line of action",
                   false)
    .set_runtime();
  DeclareParameter("contact_ratio", &contact_ratio_,
                   "mean number of pairs of teeth in contact, 1 to 2; "
                   "optional, default 1.5",
                   false);
  DeclareParameter("ramp", &ramp_,
                   "fraction of the mesh over which the number of pairs in "
                   "contact changes; optional, default 0.1",
                   false);
  DeclareParameter("stiffness_table", &stiffness_points_,
                   "stiffness of the mesh against the position x in the mesh "
                   "over [0, 1]: [x0, k0, x1, k1, ...]; replaces "
                   "tooth_stiffness and contact_ratio; optional",
                   false)
    .set_row_dim(-1);
  DeclareParameter("damping", &damping_,
                   "damping of the contact along the line of action; "
                   "optional, default 0",
                   false)
    .set_runtime();
  DeclareParameter("backlash", &backlash_,
                   "circumferential backlash at the pitch circles; "
                   "optional, default 0",
                   false);
  DeclareParameter("profile_error", &profile_points_,
                   "extra thickness of the teeth along the line of action "
                   "against the position x in the mesh over [0, 1]: [x0, p0, "
                   "x1, p1, ...]; optional",
                   false)
    .set_row_dim(-1);
  DeclareParameter("initial_phase", &initial_phase_,
                   "position in the mesh at the start, 0 to 1; optional, "
                   "default 0",
                   false);
  DeclareParameter("smoothing", &smoothing_,
                   "length over which a flank starts to load; optional, "
                   "default 1e-7",
                   false);
  DeclareParameter("internal", &internal_,
                   "gear 2 is a ring gear, with gear 1 inside it; optional, "
                   "default false",
                   false);
  DeclareParameter("helix_angle", &helix_angle_,
                   "helix angle of gear 1 at the pitch circle, radians, "
                   "positive for a right hand helix; pressure_angle is then "
                   "that of the normal section; optional, default 0",
                   false);
  DeclareParameter("face_width", &face_width_,
                   "width of the teeth along the axis; needed for "
                   "tooth_geometry, oil_viscosity and the overlap of helical "
                   "teeth; optional",
                   false);
  DeclareParameter("tooth_geometry", &teeth_,
                   "the stiffness of the mesh from the shape of the teeth, "
                   "see GearTeeth; replaces tooth_stiffness, contact_ratio "
                   "and stiffness_table; optional",
                   false);
  DeclareParameter("oil_viscosity", &oil_viscosity_,
                   "viscosity of the oil, for the squeeze film between the "
                   "flanks; optional, default 0 (none)",
                   false)
    .set_runtime();
  DeclareParameter("min_film", &min_film_,
                   "thinnest film of the squeeze film; optional, default 1e-6",
                   false);
  DeclareOutput("transmission_error", &transmission_error_,
                "transmission error, the compression of the mesh along the "
                "line of action");
  DeclareOutput("mesh_force", &mesh_force_,
                "net tangential force on the pitch circles");
  DeclareOutput("force_plus", &force_plus_,
                "force of the flanks in contact on the side of positive "
                "transmission error");
  DeclareOutput("force_minus", &force_minus_,
                "force of the flanks in contact on the side of negative "
                "transmission error");
  DeclareOutput("stiffness", &stiffness_,
                "stiffness of the mesh at its position");
  DeclareOutput("phase", &phase_, "position in the mesh cycle, 0 to 1");
  DeclareOutput("radial_force", &radial_force_, "force separating the gears");
  DeclareOutput("axial_force", &axial_force_,
                "force along the axes, of helical teeth");
  DeclareOutput("squeeze_force", &squeeze_force_,
                "force of the squeeze of the oil film");
}

shift::core::force_module::GearMesh::Attachment
shift::core::force_module::GearMesh::Resolve(
  const std::vector<std::string>& names, const std::string& what) const {
  const std::string prefix = "Gear mesh " + get_name() + ": ";
  if(names.size() != 2) {
    throw shift::core::Exception(prefix + what + " must be [body, frame]");
  }
  Attachment a;
  a.body = context_->FindActiveBody(names[0], get_parent_assembly());
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
  return a;
}

void shift::core::force_module::GearMesh::Connect() {
  gear_1_ = Resolve(gear_1_names_, "gear_1");
  gear_2_ = Resolve(gear_2_names_, "gear_2");
  if(gear_1_.body == gear_2_.body) {
    throw shift::core::Exception("Gear mesh " + get_name() +
                                 ": the gears must be on different bodies");
  }
}

ksl_SE3_t shift::core::force_module::GearMesh::Pose(
  const Attachment& a) const {
  if(a.body == context_->get_fixed_body()) {
    return *a.frame->get_d_wrt_origin();
  }
  const ksl_SE3_t origin =
    context_->get_body_origin_pose(a.body, context_->get_dhva());
  ksl_SE3_t pose;
  ksl_product_dd(&origin, a.frame->get_d_wrt_origin(), &pose);
  return pose;
}

/* 0 below 0, 1 above 1, and a cubic between with no slope at either end */
static double SmoothStep(double t) {
  if(!(t > 0.0)) {
    return 0.0;
  }
  if(t >= 1.0) {
    return 1.0;
  }
  return t * t * (3.0 - 2.0 * t);
}

double shift::core::force_module::GearMesh::PairsInContact(
  double x, double contact_ratio, double ramp) {
  /* two pairs for the fraction h on each side of x = 0 (a mesh begins with
  a new pair meeting as the old one leaves) */
  const double h = 0.5 * (contact_ratio - 1.0);
  if(h <= 0.0) {
    return 1.0;
  }
  if(h >= 0.5) {
    return 2.0;
  }
  const double f = x - std::floor(x);
  const double u = std::min(f, 1.0 - f);
  if(!(ramp > 0.0)) {
    return u < h ? 2.0 : 1.0;
  }
  return 2.0 - SmoothStep((u - h + 0.5 * ramp) / ramp);
}

void shift::core::force_module::GearMesh::BuildStiffnessTable(
  const std::function<double(double)>& spur, int points) {
  /* a helical gear is a stack of thin spur gears, each turned from the last:
  the sections across the face are spread over the overlap ratio, in tooth
  pitches */
  const double overlap = face_width_ * std::fabs(std::tan(helix_angle_)) *
                         teeth_1_ / (2.0 * M_PI * pitch_radius_1_);
  const int slices = overlap > 0.0 ? 24 : 1;
  std::vector<double> pairs;
  pairs.reserve(2 * (points + 1));
  for(int i = 0; i <= points; i++) {
    const double x = static_cast<double>(i) / points;
    double sum = 0.0;
    for(int j = 0; j < slices; j++) {
      sum += spur(x + overlap * ((j + 0.5) / slices - 0.5));
    }
    pairs.push_back(x);
    pairs.push_back(sum / slices);
  }
  stiffness_table_ = Table::FromPairs(pairs, Table::Extrapolation::kHold,
                                      "stiffness of the mesh");
}

double shift::core::force_module::GearMesh::StiffnessAt(double x) const {
  const double f = x - std::floor(x);
  if(!stiffness_table_.empty()) {
    return stiffness_table_.Evaluate(f);
  }
  return tooth_stiffness_ * PairsInContact(f, contact_ratio_, ramp_);
}

/* a table over one mesh must begin at 0 and end at 1 */
static void SpanCheck(const std::vector<double>& pairs,
                      const std::string& what) {
  if(pairs.size() < 4 || pairs.size() % 2 != 0 ||
     std::fabs(pairs.front()) > 1e-12 ||
     std::fabs(pairs[pairs.size() - 2] - 1.0) > 1e-12) {
    throw shift::core::Exception(what + " must be pairs [x, value] from x = 0 "
                                 "to x = 1 (the table spans one mesh)");
  }
}

void shift::core::force_module::GearMesh::Load() {
  const std::string prefix = "Gear mesh " + get_name() + ": ";
  if(!(pitch_radius_1_ > 0.0) || !(pitch_radius_2_ > 0.0) ||
     teeth_1_ < 1 || !(pressure_angle_ > 0.0) ||
     !(pressure_angle_ < 0.5 * M_PI) || damping_ < 0.0 || backlash_ < 0.0 ||
     !(smoothing_ > 0.0)) {
    throw shift::core::Exception(
      prefix + "pitch_radius_1, pitch_radius_2, teeth_1 and smoothing must be "
               "positive, pressure_angle between 0 and 90 degrees, damping and "
               "backlash not negative");
  }
  if(!(std::fabs(helix_angle_) < 0.5 * M_PI) || face_width_ < 0.0 ||
     oil_viscosity_ < 0.0 || !(min_film_ > 0.0)) {
    throw shift::core::Exception(
      prefix + "helix_angle must be within 90 degrees, face_width and "
               "oil_viscosity not negative and min_film positive");
  }
  if(internal_ && !(pitch_radius_2_ > pitch_radius_1_)) {
    throw shift::core::Exception(
      prefix + "the ring gear, gear 2, must be larger than gear 1");
  }
  const bool geometry = teeth_.youngs_modulus > 0.0;
  if((geometry || oil_viscosity_ > 0.0) && !(face_width_ > 0.0)) {
    throw shift::core::Exception(
      prefix + "tooth_geometry and oil_viscosity need the face_width");
  }
  /* the sections of the teeth: the normal one, along the tooth's surface, and
  the transverse one, perpendicular to the axis */
  const double beta = helix_angle_;
  transverse_angle_ =
    std::atan(std::tan(pressure_angle_) / std::cos(beta));
  const double base_helix =
    std::atan(std::tan(beta) * std::cos(transverse_angle_));
  contact_length_ = face_width_ / std::cos(base_helix);
  const double side = internal_ ? -1.0 : 1.0;
  tangential_ = std::cos(pressure_angle_) * std::cos(beta);
  axial_ = -side * std::cos(pressure_angle_) * std::sin(beta);
  radial_ = std::sin(pressure_angle_);
  const double r_1 = pitch_radius_1_ * std::sin(transverse_angle_);
  const double r_2 = pitch_radius_2_ * std::sin(transverse_angle_);
  film_radius_ = r_1 * r_2 / (r_2 + side * r_1) / std::cos(base_helix);

  if(geometry) {
    const double teeth_2 = teeth_1_ * pitch_radius_2_ / pitch_radius_1_;
    if(std::fabs(teeth_2 - std::round(teeth_2)) > 1e-6) {
      throw shift::core::Exception(
        prefix + "the pitch radii do not give a whole number of teeth on "
                 "gear 2");
    }
    teeth_.Configure(pitch_radius_1_, pitch_radius_2_, teeth_1_,
                     static_cast<int>(std::round(teeth_2)), transverse_angle_,
                     2.0 * pitch_radius_1_ / teeth_1_ * std::cos(beta),
                     internal_);
    contact_ratio_ = teeth_.get_contact_ratio();
    stiffness_points_.clear();
    BuildStiffnessTable([&](double x) {
      return teeth_.MeshStiffness(x, contact_length_);
    }, teeth_.points);
  } else if(stiffness_points_.empty()) {
    if(!(tooth_stiffness_ > 0.0) || !(contact_ratio_ >= 1.0) ||
       !(contact_ratio_ <= 2.0) || ramp_ < 0.0) {
      throw shift::core::Exception(
        prefix + "give tooth_stiffness (positive) and a contact_ratio from 1 "
                 "to 2, or a stiffness_table");
    }
    const double h = 0.5 * (contact_ratio_ - 1.0);
    if(h > 0.0 && h < 0.5 &&
       (ramp_ > 2.0 * h * (1.0 + 1e-12) ||
        ramp_ > (1.0 - 2.0 * h) * (1.0 + 1e-12))) {
      throw shift::core::Exception(
        prefix + "ramp must not exceed the fraction of the mesh with two "
                 "pairs in contact (" + std::to_string(2.0 * h) +
        ") nor that with one (" + std::to_string(1.0 - 2.0 * h) + ")");
    }
    stiffness_table_ = Table();
    if(face_width_ > 0.0 && helix_angle_ != 0.0) {
      BuildStiffnessTable([&](double x) {
        return tooth_stiffness_ * PairsInContact(x, contact_ratio_, ramp_);
      }, 200);
    }
  } else {
    SpanCheck(stiffness_points_, prefix + "stiffness_table");
    stiffness_table_ = Table::FromPairs(stiffness_points_,
                                        Table::Extrapolation::kHold,
                                        prefix + "stiffness_table");
  }
  if(!profile_points_.empty()) {
    SpanCheck(profile_points_, prefix + "profile_error");
  }
  profile_table_ = profile_points_.empty()
                     ? Table()
                     : Table::FromPairs(profile_points_,
                                        Table::Extrapolation::kHold,
                                        prefix + "profile_error");
  gear_1_.load_index = gear_2_.load_index = -2;
  const ksl_SE3_t d1 = Pose(gear_1_), d2 = Pose(gear_2_);
  ksl_vec3_t axis = d1.R.v2, diff;
  ksl_subtract_vv(&d2.t, &d1.t, &diff);
  const double along = ksl_dot_vv(&diff, &axis);
  ksl_axpy_vv(-along, &axis, &diff);
  center_distance_0_ = ksl_vec3_l2norm(&diff);
  axial_offset_0_ = along;
  if(!(center_distance_0_ > 0.0)) {
    throw shift::core::Exception(prefix + "the gears' axes coincide");
  }
  equation_count_ = 2; // the pitch point's displacement, gear 1's rotation
  q_0_ = {0.0, 0.0};
}

void shift::core::force_module::GearMesh::Service(const double* y,
                                                  double* dy) {
  Compute(y, dy, true);
}

void shift::core::force_module::GearMesh::UpdateOutputs(double,
                                                        const double* y) {
  Compute(y, nullptr, false);
}

/* e for e > 0 made smooth over w, zero otherwise, and its slope */
static double Pos(double e, double w, double* slope) {
  if(!(e > 0.0)) {
    *slope = 0.0;
    return 0.0;
  }
  if(e < w) {
    *slope = e / w;
    return e * e / (2.0 * w);
  }
  *slope = 1.0;
  return e - 0.5 * w;
}

double shift::core::force_module::GearMesh::SqueezeForce(double e,
                                                         double rate) const {
  if(!(oil_viscosity_ > 0.0) || !(rate > 0.0)) {
    return 0.0;
  }
  double slope;
  const double film = Pos(-e, smoothing_, &slope) + min_film_;
  return 3.0 * std::sqrt(2.0) * M_PI * oil_viscosity_ * contact_length_ *
         std::pow(film_radius_ / film, 1.5) * rate;
}

void shift::core::force_module::GearMesh::Compute(const double* y, double* dy,
                                                  bool apply) {
  Attachment* ends[2] = {&gear_1_, &gear_2_};
  for(Attachment* a : ends) {
    if(a->load_index == -2) {
      a->load_index = context_->GetBodyLoadIndex(a->body);
    }
  }
  const ksl_SE3_t d1 = Pose(gear_1_), d2 = Pose(gear_2_);
  const ksl_screw_t s1 = context_->GetBodyVelocity(gear_1_.load_index);
  const ksl_screw_t s2 = context_->GetBodyVelocity(gear_2_.load_index);
  const ksl_vec3_t axis = d1.R.v2;

  /* the line of centers c and the tangent of the pitch circles n, the pitch
  point P and its velocity as part of each gear (spatial velocities are
  referred to the fixed origin, v(p) = v + w x p) */
  ksl_vec3_t perp;
  ksl_subtract_vv(&d2.t, &d1.t, &perp);
  ksl_axpy_vv(-ksl_dot_vv(&perp, &axis), &axis, &perp);
  const double distance = ksl_vec3_l2norm(&perp);
  ksl_vec3_t c = perp;
  ksl_vec3_scale(&c, 1.0 / distance);
  ksl_vec3_t n;
  ksl_cross_vv(&axis, &c, &n);
  /* outside gear 1 toward gear 2, or away from it inside a ring */
  const double side = internal_ ? -1.0 : 1.0;
  ksl_vec3_t point = d1.t;
  ksl_axpy_vv(side * pitch_radius_1_ / (pitch_radius_2_ + side * pitch_radius_1_),
              &perp, &point);
  ksl_vec3_t w1_p, w2_p, v1, v2, dv;
  ksl_cross_vv(&s1.ang, &point, &w1_p);
  ksl_cross_vv(&s2.ang, &point, &w2_p);
  for(int i = 0; i < 3; i++) {
    v1.at[i] = s1.lin.at[i] + w1_p.at[i];
    v2.at[i] = s2.lin.at[i] + w2_p.at[i];
    dv.at[i] = v1.at[i] - v2.at[i];
  }
  const double tangential_rate = ksl_dot_vv(&dv, &n);
  const double approach_rate = side * ksl_dot_vv(&dv, &c); // into the teeth
  const double axial_rate = ksl_dot_vv(&dv, &axis);

  const double e_t = y[state_offset_];
  const double theta = y[state_offset_ + 1];
  if(dy) {
    dy[state_offset_] = tangential_rate;
    dy[state_offset_ + 1] = ksl_dot_vv(&s1.ang, &axis);
  }
  const double turns = teeth_1_ * theta / (2.0 * M_PI) + initial_phase_;
  const double x = turns - std::floor(turns);
  phase_ = x;
  stiffness_ = StiffnessAt(x);
  const double error = profile_table_.empty() ? 0.0 : profile_table_.Evaluate(x);

  /* the compression of each flank along its line of action, and its rate;
  gear 1 relative to gear 2 along the axis, for helical teeth */
  ksl_vec3_t between;
  ksl_subtract_vv(&d2.t, &d1.t, &between);
  const double axial = axial_offset_0_ - ksl_dot_vv(&between, &axis);
  const double approach = side * (center_distance_0_ - distance);
  const double clearance = 0.5 * tangential_ * backlash_;
  const double along = tangential_ * e_t + axial_ * axial;
  const double along_rate = tangential_ * tangential_rate + axial_ * axial_rate;
  const double e_plus = along + radial_ * approach + error - clearance;
  const double e_minus = -along + radial_ * approach + error - clearance;
  const double rate_plus = along_rate + radial_ * approach_rate;
  const double rate_minus = -along_rate + radial_ * approach_rate;
  double slope_plus, slope_minus;
  const double pos_plus = Pos(e_plus, smoothing_, &slope_plus);
  const double pos_minus = Pos(e_minus, smoothing_, &slope_minus);
  const double squeeze_plus = SqueezeForce(e_plus, rate_plus);
  const double squeeze_minus = SqueezeForce(e_minus, rate_minus);
  const double f_plus =
    std::max(0.0, stiffness_ * pos_plus + damping_ * slope_plus * rate_plus) +
    squeeze_plus;
  const double f_minus =
    std::max(0.0, stiffness_ * pos_minus + damping_ * slope_minus * rate_minus) +
    squeeze_minus;

  transmission_error_ = e_t;
  force_plus_ = f_plus;
  force_minus_ = f_minus;
  squeeze_force_ = squeeze_plus + squeeze_minus;
  mesh_force_ = (f_plus - f_minus) * tangential_;
  radial_force_ = (f_plus + f_minus) * radial_;
  axial_force_ = (f_plus - f_minus) * axial_;

  if(!apply) {
    return;
  }
  /* the force on gear 2 at the pitch point: along the tangent, and apart
  along the line of centers; the opposite on gear 1 */
  ksl_coscrew_t load;
  for(int i = 0; i < 3; i++) {
    load.lin.at[i] = mesh_force_ * n.at[i] + side * radial_force_ * c.at[i] +
                     axial_force_ * axis.at[i];
  }
  ksl_cross_vv(&point, &load.lin, &load.ang);
  context_->AddBodyLoad(gear_2_.load_index, load);
  for(int i = 0; i < 6; i++) {
    load.at[i] = -load.at[i];
  }
  context_->AddBodyLoad(gear_1_.load_index, load);
}
