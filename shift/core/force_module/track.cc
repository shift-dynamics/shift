#include "shift/core/force_module/track.h"

#include <algorithm>
#include <cmath>

#include "ksl/SE3.h"
#include "shift/core/body/base.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/collision_geometry/gjk.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"

bool shift::core::force_module::Track::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "track", shift::core::force_module::Track::Create);

std::unique_ptr<shift::core::force_module::Track>
shift::core::force_module::Track::Create() {
  return std::make_unique<shift::core::force_module::Track>();
}

void shift::core::force_module::TrackSpan::DeclareParameters() {
  DeclareParameter("name", &name, "name of the span, of its output "
                                  "<name>_sag",
                   true);
  DeclareParameter("supports", &supports,
                   "the two bodies (wheels or rollers) at the span's ends",
                   true)
    .set_row_dim(-1);
  DeclareParameter("radii", &radii,
                   "distance from each support's origin to the bushings of "
                   "the links on it",
                   true)
    .set_row_dim(-1);
}

void shift::core::force_module::Track::DeclareParameters() {
  DeclareParameter("frame", &frame_name_,
                   "body whose coordinates the outputs are measured in, "
                   "the track frame; the fixed body by default",
                   false);
  DeclareParameter("links", &link_names_, "the links of the track", true)
    .set_row_dim(-1);
  DeclareParameter("guides", &guide_names_,
                   "the rollers whose flanges guide the links, for the side "
                   "force",
                   false)
    .set_row_dim(-1);
  DeclareParameter("up", &up_,
                   "up in the frame's coordinates, for the sag; defaults "
                   "to [0, 0, -1] (z down)",
                   false)
    .set_row_dim(3);
  DeclareParameter("lateral", &lateral_,
                   "along the axles in the frame's coordinates, for the "
                   "misguide and the side force; defaults to [0, 1, 0]",
                   false)
    .set_row_dim(3);
  DeclareParameter("center", &center_,
                   "of the middle of the track from the frame's origin, "
                   "along lateral; defaults to 0",
                   false);
  DeclareParameter("spans", &spans_, "the spans whose sag is logged", false);
}

void shift::core::force_module::Track::Connect() {}

static ksl_vec3_t UnitVector(const std::vector<double>& v,
                             const std::string& what) {
  if(v.size() != 3) {
    throw shift::core::Exception(what + " must have 3 components");
  }
  ksl_vec3_t u = {{v[0], v[1], v[2]}};
  const double length = ksl_vec3_l2norm(&u);
  if(!(length > 0.0)) {
    throw shift::core::Exception(what + " must not be zero");
  }
  ksl_vec3_scale(&u, 1.0 / length);
  return u;
}

void shift::core::force_module::Track::Load() {
  const std::string prefix = "Track force module '" + name_ + "': ";
  auto find = [&](const std::string& name) {
    auto body = context_->FindActiveBody(name, get_parent_assembly());
    if(!body) {
      throw shift::core::Exception(prefix + "body '" + name +
                                   "' is not an active body");
    }
    return body;
  };
  frame_ = find(frame_name_);
  if(link_names_.empty()) {
    throw shift::core::Exception(prefix + "needs its links");
  }
  links_.clear();
  for(const auto& name : link_names_) {
    links_.push_back(find(name));
  }
  guides_.clear();
  for(const auto& name : guide_names_) {
    guides_.push_back(find(name));
  }
  UnitVector(up_, prefix + "up");
  UnitVector(lateral_, prefix + "lateral");
  supports_.clear();
  for(const auto& span : spans_.get_data()) {
    if(span->supports.size() != 2 || span->radii.size() != 2) {
      throw shift::core::Exception(prefix + "span '" + span->name +
                                   "' needs two supports and their radii");
    }
    supports_.push_back({find(span->supports[0]), find(span->supports[1])});
  }
  DeclareOutputs(OutputNames(), &outputs_,
                 "an output of the track (see its description)");
}

void shift::core::force_module::Track::Service(const double*, double*) {}

std::vector<std::string>
shift::core::force_module::Track::OutputNames() const {
  std::vector<std::string> names;
  /* get_data is not const; reading the spans does not change them */
  for(const auto& span :
      const_cast<parameter::Array<TrackSpan>&>(spans_).get_data()) {
    names.push_back(span->name + "_sag");
  }
  names.push_back("misguide");
  names.push_back("side_force");
  return names;
}

void shift::core::force_module::Track::UpdateOutputs(double, const double*) {
  /* in place: the outputs point into outputs_ */
  const std::vector<double> measured = Measure();
  assert(measured.size() == outputs_.size());
  std::copy(measured.begin(), measured.end(), outputs_.begin());
}

std::vector<double> shift::core::force_module::Track::Measure() {
  const ksl_vec3_t up = UnitVector(up_, "up");
  const ksl_vec3_t lateral = UnitVector(lateral_, "lateral");
  const ksl_SE3_t d_frame =
    context_->get_body_origin_pose(frame_, context_->get_dhva());
  /* a body's origin in the frame's coordinates */
  auto origin = [&](body::Base* body) {
    const ksl_SE3_t d = context_->get_body_origin_pose(body,
                                                       context_->get_dhva());
    ksl_vec3_t p;
    ksl_product_dinvv(&d_frame, &d.t, &p);
    return p;
  };
  std::vector<ksl_vec3_t> bushings;
  for(auto link : links_) {
    bushings.push_back(origin(link));
  }
  std::vector<double> out;

  /* sag below the line through the tops of the supports */
  const auto& spans = spans_.get_data();
  for(size_t s = 0; s < spans.size(); s++) {
    ksl_vec3_t top[2];
    for(int k = 0; k < 2; k++) {
      top[k] = origin(supports_[s][k]);
      ksl_axpy_vv(spans[s]->radii[k], &up, &top[k]);
    }
    /* the line, and its direction across up: a link is between the
    supports where its projection on the line is */
    ksl_vec3_t along;
    ksl_subtract_vv(&top[1], &top[0], &along);
    const double length2 = ksl_dot_vv(&along, &along);
    double sag = 0.0;
    for(const auto& p : bushings) {
      ksl_vec3_t w;
      ksl_subtract_vv(&p, &top[0], &w);
      const double t = length2 > 0.0 ? ksl_dot_vv(&w, &along) / length2 : -1.0;
      if(t <= 0.0 || t >= 1.0) {
        continue;
      }
      /* the line's point above or below the link, along up */
      const double a_up = ksl_dot_vv(&along, &up);
      const double w_up = ksl_dot_vv(&w, &up);
      ksl_vec3_t across = along, w_across = w;
      ksl_axpy_vv(-a_up, &up, &across);
      ksl_axpy_vv(-w_up, &up, &w_across);
      const double across2 = ksl_dot_vv(&across, &across);
      if(!(across2 > 0.0)) {
        continue; // a vertical span
      }
      const double u = ksl_dot_vv(&w_across, &across) / across2;
      /* the link's height above the line; links below the line through
      the supports' centres are on the other run */
      const double height = w_up - u * a_up;
      const double radius =
        (1.0 - u) * spans[s]->radii[0] + u * spans[s]->radii[1];
      if(height + radius <= 0.0) {
        continue;
      }
      sag = std::max(sag, -height);
    }
    out.push_back(sag);
  }

  /* misguide: the largest lateral offset from the middle */
  double misguide = 0.0;
  for(const auto& p : bushings) {
    misguide = std::max(misguide, std::fabs(ksl_dot_vv(&p, &lateral) - center_));
  }
  out.push_back(misguide);

  /* side force on the guides */
  ksl_vec3_t lateral_world;
  ksl_product_drv(&d_frame.R, &lateral, &lateral_world);
  auto is_in = [](const std::vector<body::Base*>& list, body::Base* body) {
    return std::find(list.begin(), list.end(), body) != list.end();
  };
  double side = 0.0;
  for(auto pair : context_->get_contacts()) {
    body::Base* a = pair->a->get_parent_body();
    body::Base* b = pair->b->get_parent_body();
    if((is_in(links_, a) && is_in(guides_, b)) ||
       (is_in(links_, b) && is_in(guides_, a))) {
      side += std::fabs(ksl_dot_vv(&pair->force, &lateral_world));
    }
  }
  out.push_back(side);
  return out;
}

void shift::core::force_module::Track::CheckUniqueNames() {
  const std::string prefix = FullyQualifiedName(get_name()) + "__";
  std::vector<std::string> names;
  for(const auto& span : spans_.get_data()) {
    if(!span->name.empty()) {
      names.push_back(prefix + span->name);
    }
  }
  CheckNames(names, "Track span");
}
