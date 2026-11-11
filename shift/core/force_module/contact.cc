#include "shift/core/force_module/contact.h"

#include <algorithm>
#include <cmath>

#include "shift/core/body/base.h"
#include "shift/core/body/free.h"
#include "shift/core/collision_geometry/cad.h"
#include "shift/core/collision_geometry/capsule.h"
#include "shift/core/collision_geometry/convex_hull.h"
#include "shift/core/collision_geometry/cylinder.h"
#include "shift/core/collision_geometry/ellipsoid.h"
#include "shift/core/collision_geometry/gjk.h"
#include "shift/core/collision_geometry/hertz.h"
#include "shift/core/collision_geometry/plane.h"
#include "shift/core/collision_geometry/sphere.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/joint/base.h"
#include "shift/core/log.h"

bool shift::core::force_module::Contact::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "contact", shift::core::force_module::Contact::Create);

std::unique_ptr<shift::core::force_module::Contact>
shift::core::force_module::Contact::Create() {
  return std::make_unique<shift::core::force_module::Contact>();
}

void shift::core::force_module::ContactMaterial::DeclareParameters() {
  DeclareParameter("name", &name, "material name, referred to by collision "
                                  "geometry",
                   true);
  DeclareParameter("restitution", &restitution,
                   "coefficient of restitution, 0 (no rebound) to 1 "
                   "(elastic), from which the damping follows unless given",
                   false);
  DeclareParameter("damping", &damping,
                   "the largest damping coefficient (force per velocity) of "
                   "the contact law, reached at the damping_depth of the "
                   "contact module; takes the place of restitution",
                   false);
  DeclareParameter("friction", &friction, "Coulomb friction coefficient",
                   true);
  DeclareParameter("youngs_modulus", &youngs_modulus,
                   "Young's modulus, for Hertz contact of curved surfaces "
                   "(spheres, ellipsoids, cylinders, capsules, planes)",
                   false);
  DeclareParameter("poisson_ratio", &poisson_ratio,
                   "Poisson's ratio, for Hertz contact; defaults to 0.3",
                   false);
  DeclareParameter("contact_stiffness", &contact_stiffness,
                   "force per penetration depth, for contact with convex "
                   "hulls (a linear spring)",
                   false);
  DeclareParameter("wear_coefficient", &wear_coefficient,
                   "Archard's wear coefficient over the hardness, K / H: the "
                   "volume worn per normal force and sliding distance (1/Pa "
                   "in SI units); 0 (the default) for no wear",
                   false);
  DeclareParameter("layer_thickness", &layer_thickness,
                   "thickness of an elastic layer of the material on a rigid "
                   "backing, as a brake lining: the area contact of conformal "
                   "faces (CAD faces of the same shape) presses it as an "
                   "elastic foundation, pressure E (1 - nu) / ((1 + nu) (1 - "
                   "2 nu) h) per depth. 0 (the default) for a solid body, "
                   "whose foundation is that of a flat punch on a half space "
                   "of the size of the contact",
                   false);
}

void shift::core::force_module::ContactMaterialPair::DeclareParameters() {
  DeclareParameter("materials", &materials,
                   "the names of the two materials of the pair", true);
  DeclareParameter("restitution", &restitution,
                   "coefficient of restitution of the pair", false);
  DeclareParameter("damping", &damping, "damping coefficient of the pair",
                   false);
  DeclareParameter("friction", &friction, "friction coefficient of the pair",
                   false);
  DeclareParameter("contact_stiffness", &contact_stiffness,
                   "contact stiffness of the pair, for convex hulls", false);
}

void shift::core::force_module::BristleModel::DeclareParameters() {
  DeclareParameter("bristle_stiffness", &bristle_stiffness,
                   "sigma_0, the stiffness of the bristles per unit of normal "
                   "force (1/m): the sliding at which the friction of a body "
                   "at rest reaches the friction coefficient",
                   true);
  DeclareParameter("bristle_damping", &bristle_damping,
                   "sigma_1, the damping of the bristles per unit of normal "
                   "force (s/m). With 0, a body held on several points can "
                   "rock, slip a little on each rock and creep down a slope "
                   "below the limit of holding; a body near the limit may "
                   "need more than the default; optional, default 5",
                   false);
  DeclareParameter("viscous_friction", &viscous_friction,
                   "sigma_2, friction in proportion to the sliding speed, per "
                   "unit of normal force (s/m); optional, default 0",
                   false);
  DeclareParameter("stribeck_velocity", &stribeck_velocity,
                   "sliding speed over which the friction falls from the "
                   "static to the Coulomb; optional, default 0.01",
                   false);
  DeclareParameter("points", &points,
                   "number of contact points that hold their own bristles "
                   "(the k-th point of the contact has the k-th); optional, "
                   "default 8",
                   false);
  DeclareParameter("release_time", &release_time,
                   "time over which the bristles of a point out of contact "
                   "relax to zero; optional, default 0.01",
                   false);
}

void shift::core::force_module::StickSlipPair::DeclareParameters() {
  BristleModel::DeclareParameters();
  DeclareParameter("geometry_a", &geometry_a,
                   "name of a collision geometry of the pair", true);
  DeclareParameter("geometry_b", &geometry_b,
                   "name of the other collision geometry of the pair", true);
  DeclareParameter("static_friction", &static_friction,
                   "mu_s, the friction coefficient of a body at rest, at "
                   "least that of the pair; optional, default the friction "
                   "of the pair",
                   false);
}

void shift::core::force_module::StickSlipAll::DeclareParameters() {
  BristleModel::DeclareParameters();
  DeclareParameter("static_ratio", &static_ratio,
                   "mu_s over the friction of the pair, at least 1; optional, "
                   "default 1",
                   false);
  DeclareParameter("pairs", &pairs,
                   "sets of bristles, the number of pairs that can hold at "
                   "once; optional, default 16",
                   false);
}

void shift::core::force_module::Contact::DeclareParameters() {
  shift::core::force_module::Base::DeclareParameters();
  DeclareParameter("materials", &materials_,
                   "contact materials, named by the collision geometry", true);
  DeclareParameter("pairs", &pairs_,
                   "properties of pairs of materials that override those "
                   "combined from the two materials",
                   false);
  DeclareParameter("stick_slip", &stick_slip_,
                   "pairs of collision geometry whose friction holds at "
                   "rest (the bristle model of stick and slip), instead of "
                   "the regularized Coulomb friction of the others",
                   false);
  DeclareParameter("stick_slip_all", &stick_slip_all_,
                   "the bristle model for every pair not in stick_slip, "
                   "sharing a fixed number of sets of bristles; optional",
                   false);
  DeclareParameter("default_material", &default_material_,
                   "material of collision geometry that names none; defaults "
                   "to the first material",
                   false);
  DeclareParameter("friction_combine", &friction_combine_,
                   "how the friction of two materials combines: "
                   "geometric_mean (default), arithmetic_mean, minimum, "
                   "maximum or product",
                   false);
  DeclareParameter("restitution_combine", &restitution_combine_,
                   "how the restitution of two materials combines, as "
                   "friction_combine; defaults to geometric_mean",
                   false);
  DeclareParameter("friction_velocity", &friction_velocity_,
                   "sliding speed below which friction is regularized "
                   "(reduced in proportion to the speed); defaults to 0.01",
                   false);
  DeclareParameter("damping_depth", &damping_depth_,
                   "penetration depth at which the damping of the contact "
                   "law is fully applied, ramped up from 0 at first touch; "
                   "defaults to 1e-4. A contact resting far shallower is "
                   "almost undamped: set it below the resting depth of stiff "
                   "contacts, such as a rim lying flat on a face",
                   false);
  DeclareParameter("disabled_pairs", &disabled_pairs_,
                   "pairs of bodies, by name, whose geometries are not tested "
                   "for contact, as [[\"link1\", \"link2\"], ...]; all other "
                   "pairs are",
                   false)
    .set_row_dim(-1)
    .set_column_dim(2);
}

double shift::core::force_module::Contact::Combine(const std::string& rule,
                                                   double a, double b) const {
  if(rule == "geometric_mean") {
    return std::sqrt(a * b);
  } else if(rule == "arithmetic_mean") {
    return 0.5 * (a + b);
  } else if(rule == "minimum") {
    return std::min(a, b);
  } else if(rule == "maximum") {
    return std::max(a, b);
  } else if(rule == "product") {
    return a * b;
  }
  throw shift::core::Exception("Contact force module '" + name_ +
                               "': unknown combine rule '" + rule + "'");
}

void shift::core::force_module::Contact::Load() {
  const std::string description = "Contact force module '" + name_ + "'";
  for(auto module : context_->get_active_force_modules()) {
    if(module != this && dynamic_cast<Contact*>(module)) {
      throw shift::core::Exception(
        description + ": a model has one contact force module, found '" +
        module->get_name() + "' too");
    }
  }
  if(friction_velocity_ <= 0.0 || damping_depth_ <= 0.0) {
    throw shift::core::Exception(
      description + ": friction_velocity and damping_depth must be positive");
  }
  group_mass_.clear();

  /* materials */
  auto& materials = materials_.get_data();
  const int count = static_cast<int>(materials.size());
  if(count == 0) {
    throw shift::core::Exception(description + " has no materials");
  }
  auto index_of = [&](const std::string& name) {
    for(int i = 0; i < count; i++) {
      if(materials[i]->name == name) {
        return i;
      }
    }
    throw shift::core::Exception(description + " has no material '" + name +
                                 "'");
  };
  for(int i = 0; i < count; i++) {
    const auto& m = *materials[i];
    if(index_of(m.name) != i) {
      throw shift::core::Exception(description + ": material '" + m.name +
                                   "' is given twice");
    }
    if(m.restitution > 1.0 || m.friction < 0.0 ||
       m.youngs_modulus < 0.0 || m.contact_stiffness < 0.0 ||
       m.poisson_ratio < 0.0 || m.poisson_ratio >= 0.5 ||
       m.layer_thickness < 0.0) {
      throw shift::core::Exception(
        description + ": material '" + m.name +
        "' needs restitution of at most 1, friction >= 0, Poisson's ratio in "
        "[0, 0.5) and nonnegative Young's modulus, contact stiffness and "
        "layer thickness");
    }
  }
  const int default_material =
    default_material_.empty() ? 0 : index_of(default_material_);

  /* the material and body of each collision geometry, and the shapes that
  each material is used for */
  std::vector<bool> hull(count, false), curved(count, false),
    plane(count, false), cad(count, false);
  std::vector<int> analytic(count, 0); // geometries other than CAD, of each
  geometry_.clear();
  wear_names_.clear();
  q_0_.clear();
  for(auto geometry : context_->get_active_collision_geometry()) {
    Geometry g;
    g.material = geometry->get_material().empty()
                   ? default_material
                   : index_of(geometry->get_material());
    g.load_index = context_->GetBodyLoadIndex(geometry->get_parent_body());
    if(materials[g.material]->wear_coefficient < 0.0) {
      throw shift::core::Exception(description + ": material '" +
                                   materials[g.material]->name +
                                   "' has a negative wear_coefficient");
    }
    if(materials[g.material]->wear_coefficient > 0.0 &&
       geometry->is_contact_enabled()) {
      g.wear = static_cast<int>(wear_names_.size());
      wear_names_.push_back(geometry->get_name());
      q_0_.push_back(0.0); // worn volume
    }
    geometry_[geometry] = g;
    equation_count_ = static_cast<unsigned int>(q_0_.size());
    if(!geometry->is_contact_enabled()) {
      continue; // e.g. the default floor; it gets no force if switched on
                // without the properties its contact law needs
    }
    if(!dynamic_cast<collision_geometry::Cad*>(geometry)) {
      analytic[g.material]++;
    }
    if(dynamic_cast<collision_geometry::ConvexHull*>(geometry)) {
      hull[g.material] = true;
    } else if(dynamic_cast<collision_geometry::Plane*>(geometry)) {
      plane[g.material] = true;
    } else if(dynamic_cast<collision_geometry::Cad*>(geometry)) {
      cad[g.material] = true;
    } else {
      curved[g.material] = true;
    }
  }

  /* the law of each pair of materials */
  laws_.assign(count * count, Law());
  for(int i = 0; i < count; i++) {
    for(int j = 0; j < count; j++) {
      const auto& a = *materials[i];
      const auto& b = *materials[j];
      Law& law = laws_[i * count + j];
      if(a.restitution >= 0.0 && b.restitution >= 0.0) {
        law.restitution =
          Combine(restitution_combine_, a.restitution, b.restitution);
      }
      if(a.damping >= 0.0 && b.damping >= 0.0) {
        law.damping = a.damping > 0.0 && b.damping > 0.0
                        ? 1.0 / (1.0 / a.damping + 1.0 / b.damping)
                        : 0.0;
      }
      law.friction = Combine(friction_combine_, a.friction, b.friction);
      if(a.youngs_modulus > 0.0 && b.youngs_modulus > 0.0) {
        law.modulus =
          1.0 / ((1.0 - a.poisson_ratio * a.poisson_ratio) / a.youngs_modulus +
                 (1.0 - b.poisson_ratio * b.poisson_ratio) / b.youngs_modulus);
      }
      if(a.contact_stiffness > 0.0 && b.contact_stiffness > 0.0) {
        law.stiffness =
          1.0 / (1.0 / a.contact_stiffness + 1.0 / b.contact_stiffness);
      }
      /* the layers in series; a material with none is far stiffer */
      auto foundation = [](const ContactMaterial& m) {
        if(!(m.layer_thickness > 0.0) || !(m.youngs_modulus > 0.0)) {
          return 0.0;
        }
        const double nu = m.poisson_ratio;
        return m.youngs_modulus * (1.0 - nu) /
               ((1.0 + nu) * (1.0 - 2.0 * nu) * m.layer_thickness);
      };
      const double fa = foundation(a), fb = foundation(b);
      if(fa > 0.0 && fb > 0.0) {
        law.foundation = 1.0 / (1.0 / fa + 1.0 / fb);
      } else {
        law.foundation = std::max(fa, fb);
      }
    }
  }
  for(const auto& pair : pairs_.get_data()) {
    if(pair->materials.size() != 2) {
      throw shift::core::Exception(description +
                                   ": a pair names two materials");
    }
    const int i = index_of(pair->materials[0]);
    const int j = index_of(pair->materials[1]);
    for(Law* law : {&laws_[i * count + j], &laws_[j * count + i]}) {
      if(pair->restitution >= 0.0) {
        law->restitution = std::min(pair->restitution, 1.0);
        law->damping = -1.0; // the pair's restitution over the materials'
      }
      if(pair->damping >= 0.0) {
        law->damping = pair->damping;
      }
      if(pair->friction >= 0.0) {
        law->friction = pair->friction;
      }
      if(pair->contact_stiffness > 0.0) {
        law->stiffness = pair->contact_stiffness;
      }
    }
  }

  /* the damping ratio of a mass on a spring that rebounds with the
  restitution r: r = exp(-zeta pi / sqrt(1 - zeta^2)) */
  for(auto& law : laws_) {
    if(law.restitution > 0.0) {
      const double log_r = std::log(law.restitution);
      law.damping_ratio = -log_r / std::sqrt(M_PI * M_PI + log_r * log_r);
    } else if(law.restitution == 0.0) {
      law.damping_ratio = 1.0; // critical
    }
  }

  /* every law that can be needed must be complete */
  for(int i = 0; i < count; i++) {
    for(int j = 0; j < count; j++) {
      const bool used_i = analytic[i] > 0 || cad[i];
      const bool used_j = analytic[j] > 0 || cad[j];
      /* two geometries other than CAD, which a contact law of their own needs */
      const bool analytic_pair =
        i == j ? analytic[i] > 1 : analytic[i] > 0 && analytic[j] > 0;
      const Law& law = laws_[i * count + j];
      const std::string names =
        "materials '" + materials[i]->name + "' and '" + materials[j]->name +
        "'";
      if(used_i && used_j && law.damping < 0.0 && law.restitution < 0.0) {
        throw shift::core::Exception(
          description + ": contact of " + names +
          " needs the damping or the restitution of both materials or of "
          "the pair");
      }
      if((hull[i] || hull[j]) && analytic_pair && law.stiffness <= 0.0) {
        throw shift::core::Exception(
          description + ": contact of " + names +
          " involves a convex hull, which needs the contact_stiffness of "
          "both materials or of the pair");
      }
      if(((curved[i] && (curved[j] || plane[j])) || (plane[i] && curved[j]) ||
          (cad[i] && used_j) || (cad[j] && used_i)) &&
         law.modulus <= 0.0) {
        throw shift::core::Exception(
          description + ": Hertz contact of " + names +
          " (spheres, ellipsoids, cylinders, capsules, planes, or anything "
          "against CAD geometry) needs the youngs_modulus of both materials");
      }
    }
  }

  /* the pairs with stick-slip friction, each with its states after the wear */
  bristles_.clear();
  bristle_index_.clear();
  {
    std::unordered_map<std::string, collision_geometry::Base*> by_name;
    for(auto geometry : context_->get_active_collision_geometry()) {
      if(geometry->is_contact_enabled()) {
        by_name[geometry->get_name()] = geometry;
      }
    }
    for(const auto& s : stick_slip_.get_data()) {
      const std::string pair_name = s->geometry_a + "_" + s->geometry_b;
      auto a = by_name.find(s->geometry_a);
      auto b = by_name.find(s->geometry_b);
      if(a == by_name.end() || b == by_name.end() || a->second == b->second) {
        throw shift::core::Exception(
          description + ": stick_slip of '" + s->geometry_a + "' and '" +
          s->geometry_b + "' needs two different collision geometries that "
                          "are enabled for contact");
      }
      if(bristle_index_.count({a->second, b->second})) {
        throw shift::core::Exception(description + ": stick_slip lists '" +
                                     s->geometry_a + "' and '" +
                                     s->geometry_b + "' twice");
      }
      Bristles bristles;
      bristles.name = pair_name;
      bristles.sigma_0 = s->bristle_stiffness;
      bristles.sigma_1 = s->bristle_damping;
      bristles.sigma_2 = s->viscous_friction;
      bristles.mu_coulomb = laws_[Find(a->second).material * count +
                                  Find(b->second).material]
                              .friction;
      bristles.mu_static =
        s->static_friction < 0.0 ? bristles.mu_coulomb : s->static_friction;
      bristles.velocity = s->stribeck_velocity;
      bristles.release_time = s->release_time;
      bristles.points = s->points;
      if(!(bristles.sigma_0 > 0.0) || bristles.sigma_1 < 0.0 ||
         bristles.sigma_2 < 0.0 || !(bristles.velocity > 0.0) ||
         !(bristles.release_time > 0.0) || bristles.points < 1) {
        throw shift::core::Exception(
          description + ": stick_slip of '" + s->geometry_a + "' and '" +
          s->geometry_b + "': bristle_stiffness, stribeck_velocity and "
          "release_time must be positive, bristle_damping and viscous_friction "
          "not negative, points at least 1");
      }
      if(bristles.mu_static < bristles.mu_coulomb) {
        throw shift::core::Exception(
          description + ": stick_slip of '" + s->geometry_a + "' and '" +
          s->geometry_b + "': static_friction is less than the friction (" +
          std::to_string(bristles.mu_coulomb) + ") of the pair");
      }
      bristles.offset = q_0_.size();
      q_0_.resize(q_0_.size() + 3 * bristles.points, 0.0);
      const int index = static_cast<int>(bristles_.size());
      bristle_index_[{a->second, b->second}] = index;
      bristle_index_[{b->second, a->second}] = index;
      bristles_.push_back(bristles);
    }
    /* the sets of bristles that every other pair shares */
    shared_.clear();
    shared_index_.clear();
    const StickSlipAll& all = stick_slip_all_;
    if(all.bristle_stiffness > 0.0) {
      if(all.bristle_damping < 0.0 || all.viscous_friction < 0.0 ||
         !(all.stribeck_velocity > 0.0) || !(all.release_time > 0.0) ||
         all.points < 1 || all.pairs < 1 || !(all.static_ratio >= 1.0)) {
        throw shift::core::Exception(
          description + ": stick_slip_all: stribeck_velocity and release_time "
          "must be positive, bristle_damping and viscous_friction not "
          "negative, points and pairs at least 1 and static_ratio at least 1");
      }
      for(int i = 0; i < all.pairs; i++) {
        Bristles set;
        set.sigma_0 = all.bristle_stiffness;
        set.sigma_1 = all.bristle_damping;
        set.sigma_2 = all.viscous_friction;
        set.velocity = all.stribeck_velocity;
        set.release_time = all.release_time;
        set.points = all.points;
        set.offset = q_0_.size();
        q_0_.resize(q_0_.size() + 3 * set.points, 0.0);
        shared_.push_back(set);
      }
    }
    shared_owner_.assign(shared_.size(), {nullptr, nullptr});
    shared_touched_.assign(shared_.size(), 0);
    equation_count_ = static_cast<unsigned int>(q_0_.size());
    stick_ratios_.assign(bristles_.size(), 0.0);
  }

  for(const auto& pair : disabled_pairs_) {
    if(pair.size() != 2) {
      throw shift::core::Exception("Contact force module '" + name_ +
                                   "': each of disabled_pairs names two "
                                   "bodies");
    }
    body::Base* a = context_->FindActiveBody(pair[0]);
    body::Base* b = context_->FindActiveBody(pair[1]);
    if(!a || !b) {
      throw shift::core::Exception("Contact force module '" + name_ +
                                   "': disabled_pairs names an unknown body, '" +
                                   (a ? pair[1] : pair[0]) + "'");
    }
    context_->ExcludeContact(a, b);
  }
}

const shift::core::force_module::Contact::Geometry&
shift::core::force_module::Contact::Find(
  const collision_geometry::Base* geometry) {
  auto it = geometry_.find(geometry);
  if(it == geometry_.end()) {
    throw shift::core::Exception("Contact force module '" + name_ +
                                 "': collision geometry '" +
                                 geometry->get_name() + "' is unknown");
  }
  return it->second;
}

/* curvature of a geometry at a point in its frame: 1/radius for a sphere,
the square root of the Gaussian curvature for an ellipsoid, 0 for a plane;
false for a convex hull, which has none */
static bool Curvature(const shift::core::collision_geometry::Base* geometry,
                      const ksl_vec3_t& local, double& curvature) {
  using namespace shift::core::collision_geometry;
  if(auto sphere = dynamic_cast<const Sphere*>(geometry)) {
    curvature = 1.0 / sphere->get_radius();
    return true;
  }
  if(auto ellipsoid = dynamic_cast<const Ellipsoid*>(geometry)) {
    /* K = 1 / (a b c (x^2/a^4 + y^2/b^4 + z^2/c^4))^2 */
    const ksl_vec3_t& s = ellipsoid->get_semi_axes();
    double sum = 0.0;
    for(int k = 0; k < 3; k++) {
      const double a2 = s.at[k] * s.at[k];
      sum += local.at[k] * local.at[k] / (a2 * a2);
    }
    curvature = 1.0 / (s.x * s.y * s.z * sum);
    return true;
  }
  /* a cylinder or a capsule against a shape other than a plane: Hertz
  contact at its deepest point as if round, of its radius (against a plane
  its contact is a line, see ComputeManifold) */
  if(auto cylinder = dynamic_cast<const Cylinder*>(geometry)) {
    curvature = 1.0 / cylinder->get_radius();
    return true;
  }
  if(auto capsule = dynamic_cast<const Capsule*>(geometry)) {
    curvature = 1.0 / capsule->get_radius();
    return true;
  }
  if(dynamic_cast<const Plane*>(geometry)) {
    curvature = 0.0;
    return true;
  }
  return false;
}

static ksl_vec3_t PointVelocity(const ksl_screw_t& v, const ksl_vec3_t& p) {
  ksl_vec3_t w_cross_p;
  ksl_cross_vv(&v.ang, &p, &w_cross_p);
  return {{v.lin.x + w_cross_p.x, v.lin.y + w_cross_p.y,
           v.lin.z + w_cross_p.z}};
}

double shift::core::force_module::Contact::GroupMass(int load_index) const {
  if(load_index < 0) {
    return HUGE_VAL; // rigidly attached to the fixed body
  }
  auto found = group_mass_.find(load_index);
  return found == group_mass_.end() ? 0.0 : found->second;
}

double shift::core::force_module::Contact::PointLaw(
  const collision_geometry::ContactPoint& c, double modulus, double depth,
  double& stiffness, double& exponent, double foundation) {
  if(c.area > 0.0) {
    const double pressure =
      foundation > 0.0
        ? foundation
        : 2.0 * modulus / std::sqrt(M_PI * std::max(c.patch_area, c.area));
    stiffness = pressure * c.area;
    exponent = 1.0;
    return stiffness;
  }
  if(c.length > 0.0) {
    stiffness = 0.25 * M_PI * modulus * c.length;
    exponent = 1.0;
    return stiffness;
  }
  stiffness = 4.0 / 3.0 * modulus / std::sqrt(std::max(c.curvature, 1e-12)) *
              collision_geometry::HertzEllipseFactor(c.curvature_ratio);
  exponent = 1.5;
  return 1.5 * stiffness * std::sqrt(depth); // linearized
}

std::vector<std::string>
shift::core::force_module::Contact::get_output_names() const {
  std::vector<std::string> names;
  for(const auto& name : wear_names_) {
    names.push_back("wear_" + name);
  }
  for(const auto& bristles : bristles_) {
    names.push_back("stick_" + bristles.name);
  }
  if(!shared_.empty()) {
    names.push_back("stick_all");
    names.push_back("stick_pairs");
  }
  return names;
}

void shift::core::force_module::Contact::UpdateOutputs(double,
                                                       const double* y) {
  worn_.assign(y + state_offset_, y + state_offset_ + wear_names_.size());
  /* how near each pair is to the limit of holding: the largest bristle
  deflection, |z| sigma_0 / mu_s */
  stick_ratios_.assign(bristles_.size(), 0.0);
  for(size_t i = 0; i < bristles_.size(); i++) {
    const Bristles& b = bristles_[i];
    for(int k = 0; k < b.points; k++) {
      const double* z = y + state_offset_ + b.offset + 3 * k;
      const double size = std::sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2]);
      stick_ratios_[i] = std::max(stick_ratios_[i], size * b.sigma_0 / b.mu_static);
    }
  }
  if(!shared_.empty()) {
    double largest = 0.0;
    for(size_t i = 0; i < shared_.size(); i++) {
      const Bristles& b = shared_[i];
      if(!shared_owner_[i].first || !(b.mu_static > 0.0)) {
        continue;
      }
      for(int k = 0; k < b.points; k++) {
        const double* z = y + state_offset_ + b.offset + 3 * k;
        largest = std::max(largest,
                           std::sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2]) *
                             b.sigma_0 / b.mu_static);
      }
    }
    stick_ratios_.push_back(largest);
    stick_ratios_.push_back(static_cast<double>(get_shared_pairs()));
  }
}

const shift::core::force_module::Contact::Bristles*
shift::core::force_module::Contact::SharedBristles(const GeometryPair& pair,
                                                   double friction) {
  auto found = shared_index_.find(pair);
  if(found != shared_index_.end()) {
    shared_touched_[found->second] = 1;
    return &shared_[found->second];
  }
  /* the sets that are free have no deflection, so any will do: the first */
  for(size_t i = 0; i < shared_.size(); i++) {
    if(shared_owner_[i].first) {
      continue;
    }
    Bristles& set = shared_[i];
    set.mu_coulomb = friction;
    set.mu_static = stick_slip_all_.static_ratio * friction;
    shared_owner_[i] = pair;
    shared_touched_[i] = 1;
    shared_index_[pair] = static_cast<int>(i);
    shared_index_[{pair.second, pair.first}] = static_cast<int>(i);
    return &set;
  }
  if(!shared_warned_) {
    shared_warned_ = true;
    SHIFT_WARNING("Contact " + get_name() + ": more pairs in contact than the " +
                  std::to_string(shared_.size()) + " of stick_slip_all; the "
                  "others have regularized friction");
  }
  return nullptr;
}

/* a set goes back when its pair has left contact and its bristles have relaxed
to a deflection that holds nothing */
void shift::core::force_module::Contact::ReleaseShared(const double* y) {
  for(size_t i = 0; i < shared_.size(); i++) {
    const Bristles& b = shared_[i];
    const bool touched = shared_touched_[i];
    shared_touched_[i] = 0;
    if(!shared_owner_[i].first || touched) {
      continue;
    }
    bool relaxed = true;
    const double small = 1e-6 * b.mu_static / b.sigma_0;
    for(int k = 0; k < 3 * b.points; k++) {
      relaxed = relaxed && std::fabs(y[state_offset_ + b.offset + k]) <= small;
    }
    if(relaxed) {
      const GeometryPair pair = shared_owner_[i];
      shared_index_.erase(pair);
      shared_index_.erase({pair.second, pair.first});
      shared_owner_[i] = {nullptr, nullptr};
    }
  }
}

void shift::core::force_module::Contact::AppendOutputs(
  std::vector<double>& values) const {
  values.insert(values.end(), worn_.begin(), worn_.end());
  values.resize(values.size() + wear_names_.size() - worn_.size(), 0.0);
  values.insert(values.end(), stick_ratios_.begin(), stick_ratios_.end());
}

void shift::core::force_module::Contact::Service(const double* y,
                                                 double* dy) {
  for(size_t i = 0; i < wear_names_.size(); i++) {
    dy[state_offset_ + i] = 0.0;
  }
  /* the bristles of a point that is not in contact relax to zero; those of
  the points that are get their rates below */
  for(const auto* sets : {&bristles_, &shared_}) {
    for(const auto& b : *sets) {
      for(int i = 0; i < 3 * b.points; i++) {
        dy[state_offset_ + b.offset + i] =
          -y[state_offset_ + b.offset + i] / b.release_time;
      }
    }
  }
  if(group_mass_.empty()) {
    for(auto body : context_->get_active_articulated_bodies()) {
      group_mass_[context_->GetBodyLoadIndex(body)] += body->get_inertia().m;
    }
    for(auto body : context_->get_free_bodies()) {
      group_mass_[context_->GetBodyLoadIndex(body)] += body->get_inertia().m;
    }
    group_mass_.erase(-1);
    group_mass_[-2] = 0.0; // found, even without bodies
  }
  const int count = static_cast<int>(materials_.get_data().size());
  for(auto pair : context_->get_contacts()) {
    const Geometry& a = Find(pair->a);
    const Geometry& b = Find(pair->b);
    const Law& law = laws_[a.material * count + b.material];
    pair->force = {{0.0, 0.0, 0.0}};
    const ksl_screw_t velocity_a = context_->GetBodyVelocity(a.load_index);
    const ksl_screw_t velocity_b = context_->GetBodyVelocity(b.load_index);

    /* velocity of b relative to a at the midpoint of a contact point */
    auto relative_velocity = [&](const collision_geometry::ContactPoint& c,
                                 ksl_vec3_t& p) {
      for(int k = 0; k < 3; k++) {
        p.at[k] = 0.5 * (c.point_a.at[k] + c.point_b.at[k]);
      }
      const ksl_vec3_t va = PointVelocity(velocity_a, p);
      const ksl_vec3_t vb = PointVelocity(velocity_b, p);
      return ksl_vec3_t{{vb.x - va.x, vb.y - va.y, vb.z - va.z}};
    };

    /* the effective mass of the pair, for the damping of its restitution;
    0 if neither group moves or one has no mass */
    auto effective_mass = [&]() {
      const double m_a = GroupMass(a.load_index);
      const double m_b = GroupMass(b.load_index);
      const double mass = std::isinf(m_a)   ? m_b
                          : std::isinf(m_b) ? m_a
                                            : m_a * m_b / (m_a + m_b);
      return !(mass > 0.0) || std::isinf(mass) ? 0.0 : mass;
    };

    /* the bristles of the pair, if its friction holds at rest */
    const Bristles* bristles = nullptr;
    if(!bristle_index_.empty()) {
      auto found = bristle_index_.find({pair->a, pair->b});
      if(found != bristle_index_.end()) {
        bristles = &bristles_[found->second];
      }
    }
    if(!bristles && !shared_.empty()) {
      bristles = SharedBristles({pair->a, pair->b}, law.friction);
    }

    ksl_vec3_t p;
    /* the force of a point of contact, with normal n, F = K d^e plus
    damping c, ramped up over damping_depth; slot is the point's place in
    the contact, whose bristles hold its friction */
    auto apply = [&](const collision_geometry::ContactPoint& c,
                     const ksl_vec3_t& n, double stiffness, double exponent,
                     double damping, int slot) {
      const ksl_vec3_t relative = relative_velocity(c, p);
      const double along = ksl_dot_vv(&relative, &n);
      /* IMPACT of Adams: the damping ramps up over damping_depth by STEP,
      a cubic; the force never pulls */
      const double s = std::min(c.depth / damping_depth_, 1.0);
      const double ramp = damping * s * s * (3.0 - 2.0 * s);
      const double normal =
        c.weight *
        (stiffness * std::pow(c.depth, exponent) - ramp * along);
      if(!(normal > 0.0)) {
        return;
      }
      /* regularized Coulomb friction against the sliding velocity */
      ksl_vec3_t sliding;
      for(int k = 0; k < 3; k++) {
        sliding.at[k] = relative.at[k] - along * n.at[k];
      }
      const double friction =
        law.friction * normal /
        std::sqrt(ksl_dot_vv(&sliding, &sliding) +
                  friction_velocity_ * friction_velocity_);
      /* Archard wear of each geometry whose material wears */
      const double speed = std::sqrt(ksl_dot_vv(&sliding, &sliding));
      for(const Geometry* g : {&a, &b}) {
        if(g->wear >= 0) {
          dy[state_offset_ + g->wear] +=
            materials_.get_data()[g->material]->wear_coefficient * normal *
            speed;
        }
      }
      ksl_vec3_t tangential;
      for(int k = 0; k < 3; k++) {
        tangential.at[k] = -friction * sliding.at[k];
      }
      if(bristles && slot < bristles->points) {
        /* the bristle model (LuGre): the friction force on b is
        -F (sigma_0 z + sigma_1 dz/dt + sigma_2 u) for the deflection z in
        the tangent plane, with the rate u - sigma_0 |u| z / g(|u|) */
        const double* state = y + state_offset_ + bristles->offset + 3 * slot;
        ksl_vec3_t z = {{state[0], state[1], state[2]}};
        const double z_normal = ksl_dot_vv(&z, &n);
        for(int k = 0; k < 3; k++) {
          z.at[k] -= z_normal * n.at[k];
        }
        const double g =
          bristles->mu_coulomb +
          (bristles->mu_static - bristles->mu_coulomb) *
            std::exp(-(speed / bristles->velocity) * (speed / bristles->velocity));
        for(int k = 0; k < 3; k++) {
          const double rate = sliding.at[k] - bristles->sigma_0 * speed *
                                                z.at[k] / g;
          tangential.at[k] = -normal * (bristles->sigma_0 * z.at[k] +
                                        bristles->sigma_1 * rate +
                                        bristles->sigma_2 * sliding.at[k]);
          /* the part of z along the normal, of a tangent plane that has
          turned, relaxes */
          dy[state_offset_ + bristles->offset + 3 * slot + k] =
            rate - z_normal * n.at[k] / bristles->release_time;
        }
      }
      ksl_coscrew_t load;
      for(int k = 0; k < 3; k++) {
        load.lin.at[k] = normal * n.at[k] + tangential.at[k];
        pair->force.at[k] += load.lin.at[k];
      }
      ksl_cross_vv(&p, &load.lin, &load.ang);
      context_->AddBodyLoad(b.load_index, load);
      for(int k = 0; k < 6; k++) {
        load.at[k] = -load.at[k];
      }
      context_->AddBodyLoad(a.load_index, load);
    };

    /* points with their own laws (CAD geometry): a line of contact,
    K = pi/4 E* L and e = 1 for the length L the point carries; Hertz at
    a point (see PointLaw). The damping of the restitution is that of the
    pair's linearized stiffness, shared by the points in proportion to
    theirs */
    if(pair->point_laws) {
      auto point_law = [&](const collision_geometry::ContactPoint& c,
                           double& stiffness, double& exponent) {
        return PointLaw(c, law.modulus, damping_depth_, stiffness, exponent,
                        law.foundation);
      };
      double stiffness, exponent, total = 0.0;
      for(const auto& c : pair->manifold) {
        total += c.weight * point_law(c, stiffness, exponent);
      }
      const double damping =
        law.damping >= 0.0
          ? law.damping
          : 2.0 * law.damping_ratio * std::sqrt(total * effective_mass());
      int slot = 0;
      for(const auto& c : pair->manifold) {
        const double linear = point_law(c, stiffness, exponent);
        apply(c, c.normal, stiffness, exponent,
              total > 0.0 ? damping * linear / total : 0.0, slot++);
      }
      continue;
    }

    /* stiffness and exponent of the contact law: Hertz at the deepest point
    of curved surfaces, a linear spring at each point of the manifold of a
    convex hull */
    const ksl_vec3_t& n = pair->normal;
    double stiffness, exponent;
    double curvature_a, curvature_b;
    const bool hertz = Curvature(pair->a, pair->point_local_a, curvature_a) &&
                       Curvature(pair->b, pair->point_local_b, curvature_b);
    if(hertz) {
      const double curvature = curvature_a + curvature_b;
      if(!(curvature > 0.0)) {
        continue; // two planes
      }
      stiffness = 4.0 / 3.0 * law.modulus / std::sqrt(curvature);
      exponent = 1.5;
    } else {
      stiffness = law.stiffness;
      exponent = 1.0;
    }

    /* the largest damping coefficient, given or from the restitution with
    the pair's effective mass */
    double damping = law.damping;
    if(damping < 0.0) {
      const double mass = effective_mass();
      if(hertz) {
        const double linear = 1.5 * stiffness * std::sqrt(damping_depth_);
        damping = 2.0 * law.damping_ratio * std::sqrt(linear * mass);
      } else {
        damping = 2.0 * law.damping_ratio * std::sqrt(4.0 * stiffness * mass) /
                  4.0;
      }
    }

    if(hertz) {
      collision_geometry::ContactPoint deepest;
      deepest.depth = pair->depth;
      deepest.point_a = pair->point_a;
      deepest.point_b = pair->point_b;
      apply(deepest, n, stiffness, exponent, damping, 0);
    } else {
      int slot = 0;
      for(const auto& c : pair->manifold) {
        apply(c, n, stiffness, exponent, damping, slot++);
      }
    }
  }
  ReleaseShared(y);
}
