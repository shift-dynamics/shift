#include "shift/core/force_module/tire/base.h"
#include "shift/core/terrain/base.h"

#include <cmath>
#include <vector>

#include "ksl/coscrew.h"
#include "ksl/screw.h"
#include "ksl/vec3.h"

#include "shift/core/assembly.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"
#include "shift/core/log.h"
#include "shift/core/utility.h"

bool shift::core::force_module::tire::Base::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "tire", shift::core::force_module::tire::Base::Create);

std::unique_ptr<shift::core::force_module::tire::Base>
shift::core::force_module::tire::Base::Create() {
  return std::make_unique<shift::core::force_module::tire::Base>();
}

void shift::core::force_module::tire::Base::DeclareParameters() {
  shift::core::force_module::Base::DeclareParameters();
  DeclareParameter("tire_type", &tire_type_, "tire model: \"fiala\", \"pac2002\", \"bekker_wong\" or \"iowa\"", true);
  DeclareParameter("wheel_body", &wheel_body_name_,
                   "name of the wheel body, in the same assembly; the wheel "
                   "spins about the y axis of its origin frame",
                   true);

  DeclareParameter("fiala", &fiala_, "the Fiala tire; optional", false);
  DeclareParameter("pac2002", &pac2002_, "the PAC2002 tire", false);
  DeclareParameter("bekker_wong", &bekker_wong_, "the Bekker-Wong wheel",
                   false);
  DeclareParameter("iowa", &iowa_, "the Iowa tire; optional", false);
}

std::string shift::core::force_module::tire::Base::FindFile(
  const std::string& name) const {
  const std::string file = context_->FindFile(name);
  if(!file.empty()) {
    return file;
  }
  throw shift::core::Exception("Tire " + get_name() + ": file " + name +
                               " not found");
}

void shift::core::force_module::tire::Base::ReadJSON(Json::Value& json_value) {
  for(const std::string& key : json_value.getMemberNames()) {
    if(key != "comment" && Find(key) == nullptr) {
      throw shift::core::Exception(
        "Tire: unknown member \"" + key + "\"; the parameters of a tire are "
        "in the block of its model: \"fiala\", \"pac2002\", \"bekker_wong\" "
        "or \"iowa\"");
    }
  }
  force_module::Base::ReadJSON(json_value);
}

void shift::core::force_module::tire::Base::Load() {
  if(tire_type_ == "fiala") {
    model_ = &fiala_;
  } else if(tire_type_ == "pac2002") {
    const std::string prefix = "Tire " + get_name() + ": ";
    Pac2002& tire = pac2002_;
    if(tire.definition.empty()) {
      throw shift::core::Exception(prefix + "pac2002 requires a definition "
                                            "(.tir property file)");
    }
    Pac2002Coefficients coefficients =
      Pac2002Coefficients::Read(FindFile(tire.definition));
    bool mirror = coefficients.mirror;
    if(!tire.mounted_side.empty()) {
      if(tire.mounted_side != "left" && tire.mounted_side != "right") {
        throw shift::core::Exception(prefix + "mounted_side must be "
                                              "\"left\" or \"right\"");
      }
      mirror = mirror != ((tire.mounted_side == "left") !=
                          coefficients.measured_left);
    }
    if(mirror) {
      coefficients.Mirror();
    }
    if(coefficients.relaxation != tire.use_transient_slip) {
      SHIFT_INFO(prefix + "the property file " +
                 (coefficients.relaxation ? "enables" : "disables") +
                 " relaxation; use_transient_slip is " +
                 (tire.use_transient_slip ? "true" : "false"));
    }
    tire.Configure(
      coefficients,
      ksl_vec3_l2norm(context_->get_model_parameters().get_gravity()));
    model_ = &tire;
  } else if(tire_type_ == "bekker_wong") {
    bekker_wong_.ResolveSoil("Tire " + get_name());
    model_ = &bekker_wong_;
  } else if(tire_type_ == "iowa") {
    model_ = &iowa_;
  } else {
    throw shift::core::Exception("Tire " + get_name() + ": tire_type \"" +
                                 tire_type_ + "\" is not supported");
  }
  model_->Prepare();
  equation_count_ = model_->get_state_count();
  q_0_.assign(equation_count_, 0.0);
  force_w_ = ksl_coscrew(0, 0, 0, 0, 0, 0);
}

void shift::core::force_module::tire::Base::Connect() {
  if(parent_assembly_ptr_ == nullptr) {
    throw shift::core::Exception("No parent assembly set for tire " +
                                 get_name());
  }
  try {
    wheel_body_ = (*parent_assembly_ptr_)["bodies"][wheel_body_name_]
                    .get_data<shift::core::body::Base>();
  } catch(...) {
    wheel_body_ = nullptr;
  }
  if(wheel_body_ == nullptr) {
    throw shift::core::Exception("Tire " + get_name() + ": wheel body '" +
                                 wheel_body_name_ +
                                 "' not found in its assembly");
  }
  wheel_origin_ = nullptr;
  for(auto& frame : wheel_body_->get_frames()) {
    if(frame->is_origin()) {
      wheel_origin_ = frame.get();
    }
  }
  if(wheel_origin_ == nullptr) {
    throw shift::core::Exception("Tire " + get_name() +
                                 ": wheel body has no origin frame");
  }
  load_index_ = -2;
}

/* port of the contact search of the original Fiala implementation, steps
numbered as in the source */
void shift::core::force_module::tire::Base::UpdateContact(
  const ksl_SE3_t& hub, const ksl_screw_t& velocity) {

  contact_.in_contact = false;
  contact_.depth = 0.0;
  const int segments = model_->get_contact_segments();
  if(segments > 0) {
    UpdateSlicedContact(hub, velocity, segments);
    return;
  }
  const double r0 = model_->get_unloaded_radius();

  const ksl_vec3_t& center = hub.t;
  const ksl_vec3_t& wheel_normal = hub.R.v1; // spin axis: hub y axis

  /* the terrain is taken as the plane tangent to it at the contact point:
  a point on it and its normal into the terrain (+z, z down, for the flat
  ground at z = 0 without terrains). The contact point is not known until the
  plane is, so the plane is first taken under the wheel center, then under
  the contact point found with it. */
  const auto& terrains = context_->get_active_terrain();
  ksl_vec3_t plane_point = {{center.x, center.y, 0.0}};
  ksl_vec3_t terrain_normal = {{0.0, 0.0, 1.0}};
  auto query = [&](const ksl_vec3_t& p) {
    for(auto terrain : terrains) {
      if(terrain->Query(p, plane_point, terrain_normal)) {
        return;
      }
    }
    plane_point = {{p.x, p.y, 0.0}};
    terrain_normal = {{0.0, 0.0, 1.0}};
  };

  ksl_vec3_t x_direction, down, lowest;
  double sin_tilt = 0.0, depth = 0.0;
  const int passes = terrains.empty() ? 1 : 3;
  for(int pass = 0; pass < passes; pass++) {
    query(pass == 0 ? center : lowest);

    /* no contact if the disc center is below the terrain or farther above
    it than the radius */
    const ksl_vec3_t r = {{center.x - plane_point.x, center.y - plane_point.y,
                           center.z - plane_point.z}};
    const double height = -ksl_dot_vv(&r, &terrain_normal); // center above
    if(height <= 0.0 || height >= r0) {
      return;
    }

    /* 4: heading in the terrain plane; a wheel lying flat has no contact */
    ksl_cross_vv(&wheel_normal, &terrain_normal, &x_direction);
    sin_tilt = ksl_vec3_l2norm(&x_direction);
    if(sin_tilt < 0.01) {
      return;
    }

    /* 6: lowest point of the disc, toward the terrain */
    ksl_cross_vv(&x_direction, &wheel_normal, &down);
    lowest = center;
    for(int i = 0; i < 3; i++) {
      lowest.at[i] += r0 / sin_tilt * down.at[i];
    }
    const ksl_vec3_t q = {{lowest.x - plane_point.x, lowest.y - plane_point.y,
                           lowest.z - plane_point.z}};
    depth = ksl_dot_vv(&q, &terrain_normal);
  }
  /* a cambered wheel touches the terrain with the lower edge of its tread
  first: the middle of the rim is up to half the width times the tilt of the
  axle out of the terrain plane above it */
  const double margin = model_->get_half_width() *
                        std::fabs(ksl_dot_vv(&wheel_normal, &terrain_normal));
  if(depth + margin <= 0.0) {
    return;
  }

  /* the origin of the contact frame is the lowest point projected onto the
  terrain plane */
  ksl_vec3_normalize(&x_direction);
  SetContact(hub, velocity, x_direction, terrain_normal,
             {{lowest.x - depth * terrain_normal.x,
               lowest.y - depth * terrain_normal.y,
               lowest.z - depth * terrain_normal.z}},
             depth);
}

void shift::core::force_module::tire::Base::UpdateSlicedContact(
  const ksl_SE3_t& hub, const ksl_screw_t& velocity, int segments) {
  const double radius = model_->get_unloaded_radius();
  const ksl_vec3_t& center = hub.t;
  const ksl_vec3_t& axle = hub.R.v1;

  /* the wheel's plane: the heading, level, and the direction in it that points
  most down; a wheel lying flat has no contact */
  const ksl_vec3_t z_axis = {{0.0, 0.0, 1.0}};
  ksl_vec3_t heading, down;
  ksl_cross_vv(&axle, &z_axis, &heading);
  if(ksl_vec3_l2norm(&heading) < 0.01) {
    return;
  }
  ksl_vec3_normalize(&heading);
  ksl_cross_vv(&heading, &axle, &down);

  /* a point of the lower half of the rim, the terrain beneath it, and the
  terrain's slopes there: its normal is along (slope_x, slope_y, 1) */
  struct Sample {
    ksl_vec3_t rim;
    double ground, slope_x, slope_y;
    double depth() const {
      return rim.z - ground;
    }
  };
  const auto& terrains = context_->get_active_terrain();
  auto sample = [&](int j) {
    const double along = radius * (2.0 * j / segments - 1.0);
    const double below = std::sqrt(std::max(radius * radius - along * along, 0.0));
    Sample s;
    for(int i = 0; i < 3; i++) {
      s.rim.at[i] = center.at[i] + along * heading.at[i] + below * down.at[i];
    }
    s.ground = 0.0;
    s.slope_x = s.slope_y = 0.0;
    for(auto terrain : terrains) {
      ksl_vec3_t surface, normal;
      if(terrain->Query(s.rim, surface, normal)) {
        s.ground = surface.z;
        s.slope_x = normal.x / normal.z;
        s.slope_y = normal.y / normal.z;
        break;
      }
    }
    return s;
  };

  /* the area below the terrain of each slice, a trapezoid or a triangle,
  with sums for the centroid and the mean normal */
  double area = 0.0;
  ksl_vec3_t moment = {{0.0, 0.0, 0.0}}, normal_sum = {{0.0, 0.0, 0.0}};
  Sample a = sample(0);
  for(int j = 1; j <= segments; j++) {
    const Sample b = sample(j);
    const double d_a = a.depth(), d_b = b.depth();
    if(d_a > 0.0 || d_b > 0.0) {
      const double length = std::hypot(b.rim.x - a.rim.x, b.rim.y - a.rim.y);
      double part, along, height;
      if(d_a > 0.0 && d_b > 0.0) {
        part = 0.5 * (d_a + d_b) * length;
        along = (d_a + 2.0 * d_b) / (3.0 * (d_a + d_b));
        const double rim = a.rim.z * a.rim.z + a.rim.z * b.rim.z +
                           b.rim.z * b.rim.z;
        const double ground = a.ground * a.ground + a.ground * b.ground +
                              b.ground * b.ground;
        height = (rim - ground) / (3.0 * (d_a + d_b));
      } else {
        /* the rim crosses the terrain within the slice, at the fraction
        crossing; the triangle is on the side of the deeper end */
        const double crossing = d_a / (d_a - d_b);
        const double cross_z =
          a.ground + crossing * (b.ground - a.ground);
        const Sample& deep = d_a > 0.0 ? a : b;
        const double base = d_a > 0.0 ? crossing : 1.0 - crossing;
        part = 0.5 * deep.depth() * base * length;
        along = d_a > 0.0 ? base / 3.0 : 1.0 - base / 3.0;
        height = (deep.ground + deep.rim.z + cross_z) / 3.0;
      }
      const double slope_x = a.slope_x + along * (b.slope_x - a.slope_x);
      const double slope_y = a.slope_y + along * (b.slope_y - a.slope_y);
      const double size = std::sqrt(1.0 + slope_x * slope_x + slope_y * slope_y);
      area += part;
      moment.x += part * (a.rim.x + along * (b.rim.x - a.rim.x));
      moment.y += part * (a.rim.y + along * (b.rim.y - a.rim.y));
      moment.z += part * height;
      normal_sum.x += part * slope_x / size;
      normal_sum.y += part * slope_y / size;
      normal_sum.z += part / size;
    }
    a = b;
  }
  if(!(area > 0.0)) {
    return;
  }

  /* the half angle of the circular segment of this area, by Newton's method
  on theta - sin(2 theta) / 2 = area / r^2, from the half disc; its
  derivative, 1 - cos(2 theta), is positive between 0 and pi */
  const double target = std::min(area / (radius * radius), M_PI);
  double theta = 0.5 * M_PI;
  for(int i = 0; i < 50; i++) {
    const double step = (theta - 0.5 * std::sin(2.0 * theta) - target) /
                        (1.0 - std::cos(2.0 * theta));
    theta = std::min(std::max(theta - step, 0.5 * theta), M_PI);
    if(std::fabs(step) < 1e-10) {
      break;
    }
  }
  const double depth = radius * (1.0 - std::cos(theta));

  /* the heading in the terrain plane, and the frame at the centroid */
  ksl_vec3_normalize(&normal_sum);
  ksl_cross_vv(&axle, &normal_sum, &heading);
  if(ksl_vec3_l2norm(&heading) < 0.01) {
    return;
  }
  ksl_vec3_normalize(&heading);
  SetContact(hub, velocity, heading, normal_sum,
             {{moment.x / area, moment.y / area, moment.z / area}}, depth);
}

void shift::core::force_module::tire::Base::SetContact(
  const ksl_SE3_t& hub, const ksl_screw_t& velocity,
  const ksl_vec3_t& x_direction, const ksl_vec3_t& terrain_normal,
  const ksl_vec3_t& origin, double depth) {
  const ksl_vec3_t& center = hub.t;
  const ksl_vec3_t& wheel_normal = hub.R.v1;

  /* 15 */
  contact_.depth = depth;
  contact_.in_contact = true;

  /* 10 - 14: contact frame, x heading, z terrain normal, y = z x x */
  ksl_vec3_t y_direction;
  ksl_cross_vv(&terrain_normal, &x_direction, &y_direction);
  ksl_SE3_t& w = contact_.w_frame;
  w.R.v0 = x_direction;
  w.R.v1 = y_direction;
  w.R.v2 = terrain_normal;
  w.t = origin;

  /* 16 - 17: velocity of the wheel's material point at the contact point,
  in the w frame; the spatial velocity is referred to the fixed origin, so
  the velocity of a point p is v + w x p */
  ksl_vec3_t w_cross_p;
  ksl_cross_vv(&velocity.ang, &w.t, &w_cross_p);
  ksl_vec3_t contact_velocity = {{velocity.lin.x + w_cross_p.x,
                                  velocity.lin.y + w_cross_p.y,
                                  velocity.lin.z + w_cross_p.z}};
  contact_.contact_velocity_w = {{ksl_dot_vv(&w.R.v0, &contact_velocity),
                                  ksl_dot_vv(&w.R.v1, &contact_velocity),
                                  ksl_dot_vv(&w.R.v2, &contact_velocity)}};

  /* 20: hub velocity along the heading */
  ksl_vec3_t w_cross_c;
  ksl_cross_vv(&velocity.ang, &center, &w_cross_c);
  ksl_vec3_t hub_velocity = {{velocity.lin.x + w_cross_c.x,
                              velocity.lin.y + w_cross_c.y,
                              velocity.lin.z + w_cross_c.z}};
  contact_.vx = ksl_dot_vv(&hub_velocity, &x_direction);

  /* velocity of the contact center: the material velocity at the w frame
  origin less the spin, (w . a) a x (p - c) */
  const double spin = ksl_dot_vv(&velocity.ang, &wheel_normal);
  ksl_vec3_t arm = {{w.t.x - center.x, w.t.y - center.y, w.t.z - center.z}};
  ksl_vec3_t spin_velocity;
  ksl_cross_vv(&wheel_normal, &arm, &spin_velocity);
  ksl_vec3_t center_velocity = {{contact_velocity.x - spin * spin_velocity.x,
                                 contact_velocity.y - spin * spin_velocity.y,
                                 contact_velocity.z - spin * spin_velocity.z}};
  contact_.center_velocity_w = {{ksl_dot_vv(&w.R.v0, &center_velocity),
                                 ksl_dot_vv(&w.R.v1, &center_velocity),
                                 ksl_dot_vv(&w.R.v2, &center_velocity)}};

  /* hub velocity and angular velocity of the wheel in the w frame */
  contact_.hub_velocity_w = {{ksl_dot_vv(&w.R.v0, &hub_velocity),
                              ksl_dot_vv(&w.R.v1, &hub_velocity),
                              ksl_dot_vv(&w.R.v2, &hub_velocity)}};
  contact_.angular_velocity_w = {{ksl_dot_vv(&w.R.v0, &velocity.ang),
                                  ksl_dot_vv(&w.R.v1, &velocity.ang),
                                  ksl_dot_vv(&w.R.v2, &velocity.ang)}};

  /* the hub in the w frame */
  const ksl_vec3_t to_hub = {{center.x - w.t.x, center.y - w.t.y,
                              center.z - w.t.z}};
  contact_.hub_w = {{ksl_dot_vv(&w.R.v0, &to_hub),
                     ksl_dot_vv(&w.R.v1, &to_hub),
                     ksl_dot_vv(&w.R.v2, &to_hub)}};

  /* spin axis in the w frame */
  contact_.spin_axis_w = {{ksl_dot_vv(&w.R.v0, &wheel_normal),
                           ksl_dot_vv(&w.R.v1, &wheel_normal),
                           ksl_dot_vv(&w.R.v2, &wheel_normal)}};
}

void shift::core::force_module::tire::Base::Service(const double* y,
                                                    double* dy) {
  assert(context_ && model_ && wheel_origin_);
  if(load_index_ == -2) {
    load_index_ = context_->GetBodyLoadIndex(wheel_body_);
  }
  const ksl_SE3_t& hub = context_->get_dhva().d[wheel_origin_->get_index()];
  const ksl_screw_t velocity = context_->GetBodyVelocity(load_index_);

  UpdateContact(hub, velocity);
  /* 35: spin rate about the wheel axis */
  contact_.omega = ksl_dot_vv(&hub.R.v1, &velocity.ang);

  model_->Evaluate(contact_, y + state_offset_, dy + state_offset_, &force_w_);
  if(!contact_.in_contact) {
    return;
  }

  /* force and moment at the contact point, in the fixed frame and referred
  to its origin: (R f, R m + p x R f) */
  const ksl_SE3_t& w = contact_.w_frame;
  ksl_coscrew_t load;
  for(int i = 0; i < 3; i++) {
    load.lin.at[i] = w.R.v0.at[i] * force_w_.lin.x +
                     w.R.v1.at[i] * force_w_.lin.y +
                     w.R.v2.at[i] * force_w_.lin.z;
    load.ang.at[i] = w.R.v0.at[i] * force_w_.ang.x +
                     w.R.v1.at[i] * force_w_.ang.y +
                     w.R.v2.at[i] * force_w_.ang.z;
  }
  ksl_vec3_t p_cross_f;
  ksl_cross_vv(&w.t, &load.lin, &p_cross_f);
  for(int i = 0; i < 3; i++) {
    load.ang.at[i] += p_cross_f.at[i];
  }
  context_->AddBodyLoad(load_index_, load);
}

std::vector<std::string>
shift::core::force_module::tire::Base::get_output_names() const {
  if(tire_type_ == "iowa") {
    return {"deflection",         "slip",          "slip_angle",
            "normal_force",       "longitudinal_force", "lateral_force",
            "aligning_moment"};
  }
  if(tire_type_ != "bekker_wong") {
    return {};
  }
  return {"sinkage",    "slip",   "drawbar_pull", "load",
          "shear_torque", "camber", "side_force"};
}

void shift::core::force_module::tire::Base::UpdateOutputs(double,
                                                          const double*) {
  outputs_.clear();
  if(const auto* iowa = dynamic_cast<const Iowa*>(model_)) {
    const bool touching = contact_.in_contact;
    outputs_ = {touching ? contact_.depth : 0.0, iowa->get_slip(),
                iowa->get_slip_angle(), touching ? -force_w_.lin.z : 0.0,
                touching ? force_w_.lin.x : 0.0,
                touching ? force_w_.lin.y : 0.0,
                touching ? force_w_.ang.z : 0.0};
    return;
  }
  const auto* wheel = dynamic_cast<const BekkerWong*>(model_);
  if(wheel == nullptr) {
    return;
  }
  const bool touching = contact_.in_contact;
  outputs_ = {touching ? contact_.depth : 0.0, wheel->get_slip(),
              touching ? force_w_.lin.x : 0.0,
              touching ? -force_w_.lin.z : 0.0, wheel->get_shear_torque(),
              wheel->get_camber(), touching ? force_w_.lin.y : 0.0};
}

void shift::core::force_module::tire::Base::AppendOutputs(
  std::vector<double>& values) const {
  values.insert(values.end(), outputs_.begin(), outputs_.end());
}
