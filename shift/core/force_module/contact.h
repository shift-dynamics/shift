#ifndef SHIFT_FORCEMODULE_CONTACT_H_
#define SHIFT_FORCEMODULE_CONTACT_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "shift/core/force_module/base.h"
#include "shift/core/parameter/array.h"

namespace shift {
namespace core {

namespace collision_geometry {
class Base;
}

namespace force_module {

/*!
@brief contact properties of a material, see Contact

Values are in the model's units. Each property is needed only by the contact
laws that use it: the elastic properties by curved surfaces in contact (Hertz),
the contact stiffness by convex hulls (a linear spring).
*/
class ContactMaterial : public parameter::List {
public:
  ContactMaterial() {
    DeclareParameters();
  }
  virtual ~ContactMaterial() {}

  static const std::string get_array_key() {
    return "materials";
  }

  std::string name;
  double youngs_modulus = 0.0; //!< 0 if not given
  double poisson_ratio = 0.3;
  double restitution = -1.0;       //!< 0 to 1, negative if not given
  double damping = -1.0;           //!< force per velocity, negative if not given
  double friction = -1.0;          //!< Coulomb friction coefficient
  double contact_stiffness = 0.0;  //!< force per depth, 0 if not given

protected:
  virtual void DeclareParameters() override;
};

/*!
@brief properties of a pair of materials in contact that override those
combined from the two materials; a negative value is not given
*/
class ContactMaterialPair : public parameter::List {
public:
  ContactMaterialPair() {
    DeclareParameters();
  }
  virtual ~ContactMaterialPair() {}

  static const std::string get_array_key() {
    return "pairs";
  }

  std::vector<std::string> materials;
  double restitution = -1.0;
  double damping = -1.0;
  double friction = -1.0;
  double contact_stiffness = -1.0;

protected:
  virtual void DeclareParameters() override;
};

/*!
@brief contact forces between collision geometry, from the contacts found by
Context::DetectContacts

At each contact point with penetration depth d > 0 and rate of penetration
v = -(v_b - v_a) . n at the midpoint of the point's two ends (n is the normal
from a to b), the normal force on b is that of the IMPACT function of MSC
Adams, scaled by the point's weight w (see collision_geometry::ContactPoint):

  F = w max(0, K d^e + STEP(d, 0, 0, damping_depth, c) v)

  STEP(d, 0, 0, h, c) = c s^2 (3 - 2 s),   s = min(d / h, 1)

The damping coefficient ramps up from 0 at first touch to c at
damping_depth, so the force is continuous, with continuous first
derivatives at first touch for e > 1, and depends on the state only: a
variable step integrator sees no jump in it.

The stiffness depends on the shapes:

- spheres, ellipsoids and planes: Hertz contact at the deepest point,
  K = 4/3 E* sqrt(R*) and e = 3/2, with
  1/E* = (1 - nu_a^2)/E_a + (1 - nu_b^2)/E_b and 1/R* = k_a + k_b from the
  curvatures at the contact points (1/radius for a sphere, the square root of
  the Gaussian curvature for an ellipsoid, 0 for a plane)
- a convex hull with any shape: a linear spring, K = k and e = 1, at each
  point of the contact manifold (see collision_geometry::ComputeManifold),
  with its own depth and rate of penetration: k is the stiffness of one
  contact point, so a box on its face rests on four springs. The contact
  stiffnesses of the materials combine in series, 1/k = 1/k_a + 1/k_b, unless
  given for the pair.

The damping c of a pair is its "damping" if given for the pair, or for both
materials (combined in series, 1/c = 1/c_a + 1/c_b); otherwise it follows
from the coefficient of restitution r (combined by restitution_combine) as
the damping of a mass on a linear spring that rebounds with r,

  c = 2 zeta sqrt(w k m) / w,   zeta = -ln r / sqrt(pi^2 + ln^2 r)

with m the effective mass of the pair, m_a m_b / (m_a + m_b) of the masses
of the two rigidly joined groups of bodies (infinite for the fixed body),
and, for Hertz contact, one point (w = 1) with the stiffness linearized at
damping_depth, k = 3/2 K sqrt(damping_depth); for a hull, per unit weight
of the manifold's points, as for a box resting on its face (w = 4 in all).
This is approximate: the rebound of a Hertz contact with constant damping
depends on the impact speed; a strongly damped contact (r below about 0.5)
rebounds higher than r, as its force, which never pulls, ends before the
spring is relaxed; and bodies on joints move with an effective mass of their
own.

Friction is Coulomb friction regularized below friction_velocity, against the
sliding velocity u at the point:

  F_t = -mu F u / sqrt(|u|^2 + friction_velocity^2)

Friction and restitution of a pair are combined from the two materials by
friction_combine and restitution_combine, unless the pair is listed in pairs.
Equal and opposite forces act on a and b at the point.

Collision geometry names its material ("material"), or has the default
material. Bodies joined by a joint other than a float joint are not tested for
contact when exclude_jointed_bodies is set.

MSC Software, Adams Solver function reference: IMPACT and STEP.
*/
class Contact : public Base {
public:
  Contact()
    : Base() {
    DeclareParameters();
  }
  Contact(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Contact() override {}
  static std::unique_ptr<Contact> Create();
  void DeclareParameters() override;

  static bool is_registered() {
    return registered_;
  }

  /*! \brief resolve the materials of the collision geometry, combine the
   * properties of each pair of materials and exclude jointed bodies */
  virtual void Load() override;
  virtual void Service(const double* y, double* dy) override;
  virtual void Connect() override {}

private:
  /* the contact law of a pair of materials */
  struct Law {
    double modulus = 0.0;   //!< E*, 0 if a material has no Young's modulus
    double stiffness = 0.0; //!< k, 0 if not available
    double restitution = -1.0; //!< negative if not given
    double damping = -1.0;     //!< c, negative to follow from restitution
    double damping_ratio = 0.0; //!< zeta of the restitution
    double friction = 0.0;
  };
  struct Geometry {
    int material = 0;
    int load_index = -1;
  };

  double Combine(const std::string& rule, double a, double b) const;
  const Geometry& Find(const collision_geometry::Base* geometry);

  static bool registered_;
  parameter::Array<ContactMaterial> materials_;
  parameter::Array<ContactMaterialPair> pairs_;
  std::string default_material_;
  std::string friction_combine_ = "geometric_mean";
  std::string restitution_combine_ = "geometric_mean";
  double friction_velocity_ = 0.01;
  double damping_depth_ = 1e-4;
  bool exclude_jointed_bodies_ = true;

  std::vector<Law> laws_; //!< by material indices, laws_[i * count + j]
  std::unordered_map<const collision_geometry::Base*, Geometry> geometry_;
  /*! mass of each rigidly joined group of bodies, by body load index (see
   * Context::GetBodyLoadIndex); found at the first Service, once the mass
   * properties are loaded */
  std::unordered_map<int, double> group_mass_;
  double GroupMass(int load_index) const;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
