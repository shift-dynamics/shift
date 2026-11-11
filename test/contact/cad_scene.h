#ifndef SHIFT_TEST_CONTACT_CAD_SCENE_H_
#define SHIFT_TEST_CONTACT_CAD_SCENE_H_

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "gtest/gtest.h"
#include "ksl/SE3.h"
#include "ksl/vec3.h"
#include "shift/config.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/collision_geometry/cad.h"
#include "shift/core/context.h"
#include "shift/core/force_module/contact.h"
#include "test/contact/contact_record.h"

namespace shift {
namespace test {

/* a STEP part at the origin and a shape that can be moved */
class CadScene {
public:
  CadScene(const std::string& name, const std::string& step,
           const std::string& shape, const std::string& cad_keys = "",
           const std::string& body_geometry = "") {
    std::ostringstream m;
    m << R"({"model_parameters": {"version": "0.2",
      "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                "time": "seconds"},
      "gravity": [0, 0, 9.81], "include_directories": [")"
      << SHIFT_EXAMPLES_DIR << R"(/cad", ")" << SHIFT_EXAMPLES_DIR
      << R"(/pendulum"],
      "enable_contact_detection": true, "solvers": [{"type": "rnca"}],
      "integrators": [{"type": "ode113", "time_span": 0.0,
                       "time_step": 0.01}]},
      "assemblies": [{"name": "m", "bodies": [
      {"name": "part", "geometry": [{"type": "cad", "filename": ")"
      << step << R"("}],
       "collision_geometry": [{"name": "part", "type": "cad",
                               "material": "steel")"
      << cad_keys << R"(}],
       "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0,
                   "Iyz": 0, "Izx": 0}},
      {"name": "body", "geometry": [)" << body_geometry << R"(],
       "collision_geometry": [{"name": "shape", "material": "steel", )"
      << shape << R"(}],
       "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0,
                   "Iyz": 0, "Izx": 0}}],
      "joints": [
      {"name": "part_joint", "type": "rigid",
       "parent": [["fixed"], "origin"], "child": [["part"], "origin"]},
      {"name": "body_joint", "type": "float",
       "parent": [["fixed"], "origin"], "child": [["body"], "origin"]}],
      "force_modules": [{"name": "contact", "type": "contact",
        "materials": [{"name": "steel", "youngs_modulus": 2e11,
                       "poisson_ratio": 0.3, "restitution": 0.3,
                       "friction": 0.3}]}]}]})";
    const std::string path = testing::TempDir() + "/" + name + ".json";
    {
      std::ofstream file(path);
      file << m.str();
    }
    context_.LoadJSON(path);
    std::remove(path.c_str());
    context_.UpdateD();
    for(auto g : context_.get_active_collision_geometry()) {
      (g->get_name() == "part" ? part_ : shape_) = g;
    }
  }

  /* the contact with the shape at p, turned by the rotation r (rows); the
  points and normal are from the part to the shape, whichever order */
  const shift::core::collision_geometry::ContactPair& At(const ksl_vec3_t& p, const double r[3][3] = nullptr,
                        bool part_first = true) {
    ksl_SE3_t d;
    for(int k = 0; k < 3; k++) {
      d.t.at[k] = p.at[k];
      for(int j = 0; j < 3; j++) {
        d.R.at[j][k] = r ? r[k][j] : (j == k ? 1.0 : 0.0);
      }
    }
    const ksl_SE3_t identity = {{{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}}};
    part_->UpdatePose(identity);
    shape_->UpdatePose(d);
    pair_ = shift::core::collision_geometry::ContactPair();
    pair_.a = part_first ? part_ : shape_;
    pair_.b = part_first ? shape_ : part_;
    shift::test::TestContact(pair_);
    return pair_;
  }

  /* At, keeping what the pair kept from the last test, as from one step of a
  simulation to the next: the part first */
  const shift::core::collision_geometry::ContactPair& Follow(const ksl_vec3_t& p,
                                                            const double r[3][3] = nullptr) {
    ksl_SE3_t d;
    for(int k = 0; k < 3; k++) {
      d.t.at[k] = p.at[k];
      for(int j = 0; j < 3; j++) {
        d.R.at[j][k] = r ? r[k][j] : (j == k ? 1.0 : 0.0);
      }
    }
    const ksl_SE3_t identity = {{{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}}};
    part_->UpdatePose(identity);
    shape_->UpdatePose(d);
    if(pair_.a != part_ || pair_.b != shape_) {
      pair_ = shift::core::collision_geometry::ContactPair();
      pair_.a = part_;
      pair_.b = shape_;
    }
    shift::test::TestContact(pair_);
    return pair_;
  }

  const shift::core::collision_geometry::Cad& part() const {
    return *static_cast<const shift::core::collision_geometry::Cad*>(part_);
  }
  const shift::core::collision_geometry::Base& shape() const {
    return *shape_;
  }
  /* a point in the frame of the part's geometry */
  ksl_vec3_t ToPart(const ksl_vec3_t& p) const {
    ksl_vec3_t out;
    ksl_product_dinvv(&part_->get_d_absolute(), &p, &out);
    return out;
  }

private:
  shift::core::Context context_;
  shift::core::collision_geometry::Base* part_ = nullptr;
  shift::core::collision_geometry::Base* shape_ = nullptr;
  shift::core::collision_geometry::ContactPair pair_;
};

/* the rotation by angle about the unit axis, rows */
inline void Rotation(const double axis[3], double angle, double r[3][3]) {
  const double c = std::cos(angle), s = std::sin(angle), v = 1.0 - c;
  const double x = axis[0], y = axis[1], z = axis[2];
  const double m[3][3] = {{c + x * x * v, x * y * v - z * s, x * z * v + y * s},
                          {y * x * v + z * s, c + y * y * v, y * z * v - x * s},
                          {z * x * v - y * s, z * y * v + x * s, c + z * z * v}};
  std::copy(&m[0][0], &m[0][0] + 9, &r[0][0]);
}

/* the elastic force of a pair's points, for a modulus E* = 1 */
inline double ElasticForce(const shift::core::collision_geometry::ContactPair& pair) {
  double force = 0.0;
  for(const auto& c : pair.manifold) {
    double stiffness, exponent;
    shift::core::force_module::Contact::PointLaw(c, 1.0, 1e-4, stiffness,
                                                 exponent);
    force += c.weight * stiffness * std::pow(c.depth, exponent);
  }
  return force;
}

} // namespace test
} // namespace shift

#endif // SHIFT_TEST_CONTACT_CAD_SCENE_H_
