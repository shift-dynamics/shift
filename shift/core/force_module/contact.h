#ifndef SHIFT_FORCEMODULE_CONTACT_H_
#define SHIFT_FORCEMODULE_CONTACT_H_

#include <map>
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
struct ContactPoint;
}

namespace force_module {

/*!
@brief contact properties of a material, see Contact

Values are in the model's units. Each property is needed only by the contact
laws that use it: the elastic properties by curved surfaces in contact (Hertz) and by
CAD geometry, the contact stiffness by convex hulls (a linear spring).
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
  double wear_coefficient = 0.0;   //!< K / H of Archard, 0 for no wear

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
@brief a pair of collision geometry whose friction holds at rest: the
parameters of the bristle (LuGre) model of its stick and slip, see Contact
*/
class StickSlipPair : public parameter::List {
public:
  StickSlipPair() {
    DeclareParameters();
  }
  virtual ~StickSlipPair() {}

  static const std::string get_array_key() {
    return "stick_slip";
  }

  std::string geometry_a, geometry_b;
  double bristle_stiffness = 0.0;
  double bristle_damping = 0.0;
  double viscous_friction = 0.0;
  double static_friction = -1.0;
  double stribeck_velocity = 0.01;
  int points = 8;
  double release_time = 0.01;

protected:
  virtual void DeclareParameters() override;
};

/*!
@brief contact forces between collision geometry, from the contacts found by
Context::DetectContacts

At each contact point with penetration depth \f$d > 0\f$ and rate of
penetration \f$v = -(\underline{v}_b - \underline{v}_a) \cdot \underline{n}\f$
at the midpoint of the point's two ends (\f$\underline{n}\f$ is the normal from
a to b), the normal force on b is that of the IMPACT function of MSC Adams,
scaled by the point's weight \f$w\f$ (see collision_geometry::ContactPoint):

\f[
f = w \max\left(0, \; k d^n + \mathrm{STEP}(d, 0, 0, h, c) \, v\right)
\f]

\f[
\mathrm{STEP}(d, 0, 0, h, c) = c \, s^2 (3 - 2 s), \quad s = \min(d / h, 1)
\f]

with \f$h\f$ the damping_depth. The damping coefficient ramps up from 0 at
first touch to \f$c\f$ at damping_depth, so the force is continuous, with
continuous first derivatives at first touch for \f$n > 1\f$, and depends on the state only: a
variable step integrator sees no jump in it.

The stiffness depends on the shapes:

- spheres, ellipsoids and planes: Hertz contact at the deepest point,
  \f$k = \frac{4}{3} e^* \sqrt{r^*}\f$ and \f$n = 3/2\f$, with
  \f$1/e^* = (1 - \nu_a^2)/e_a + (1 - \nu_b^2)/e_b\f$ and
  \f$1/r^* = \kappa_a + \kappa_b\f$ from the curvatures at the contact points
  (1/radius for a sphere, the square root of the Gaussian curvature for an
  ellipsoid, 0 for a plane)
- a convex hull with any shape: a linear spring, \f$n = 1\f$ and \f$k\f$ the
  stiffness of one contact point, at each point of the contact manifold (see
  collision_geometry::ComputeManifold), with its own depth and rate of
  penetration: so a box on its
  face rests on four springs. The contact stiffnesses of the materials
  combine in series, \f$1/k = 1/k_a + 1/k_b\f$, unless given for the pair.
- CAD geometry (collision_geometry::Cad), on CAD geometry or a plane: each
  point of the contact found on the exact surfaces has its own normal and
  law (collision_geometry::ContactPoint, see PointLaw). A point where two
  curved surfaces touch is Hertz contact of an ellipse,
  \f$k = \frac{4}{3} e^* \sqrt{r^*} \, g\f$ and \f$n = 3/2\f$, with \f$1/r^*\f$
  the sum of the mean curvatures of the two surfaces there and \f$g\f$ the
  factor of the ellipse's elongation (collision_geometry::HertzEllipseFactor,
  1 for a circle). A line of contact, between cylinders or a cylinder on a
  plane, is divided into slices along its length, each the Hertz contact of
  two cylinders over its length \f$l\f$: a linear spring,
  \f$k = \frac{\pi}{4} e^* l\f$ and \f$n = 1\f$. The slices
  cover only the part of the line where the surfaces overlap, so a line
  tilting out of parallel carries its load towards one end. CAD geometry on a sphere, an
  ellipsoid or a convex hull has one point, as Hertz contact with the
  curvature of the CAD geometry's edge_radius.

The damping \f$c\f$ of a pair is its "damping" if given for the pair, or for
both materials (combined in series, \f$1/c = 1/c_a + 1/c_b\f$); otherwise it
follows from the coefficient of restitution \f$r\f$ (combined by
restitution_combine) as the damping of a mass on a linear spring that
rebounds with \f$r\f$,

\f[
c = \frac{2 \zeta \sqrt{w \bar{k} m}}{w}, \quad
\zeta = \frac{-\ln r}{\sqrt{\pi^2 + \ln^2 r}}
\f]

with \f$m\f$ the effective mass of the pair, \f$m_a m_b / (m_a + m_b)\f$ of the
masses of the two rigidly joined groups of bodies (infinite for the fixed
body), and, for Hertz contact, one point (\f$w = 1\f$) with the stiffness
linearized at the damping depth \f$h\f$, \f$\bar{k} = \frac{3}{2} k \sqrt{h}\f$; for a
hull, per unit weight of the manifold's points, as for a box resting on its
face (\f$w = 4\f$ in all); for CAD geometry, \f$\bar{k}\f$ is the sum of the points'
stiffnesses, each linearized at \f$h\f$ for \f$n = 3/2\f$, and \f$c\f$ is shared
by the points in proportion to their stiffness.
This is approximate: the rebound of a Hertz contact with constant damping
depends on the impact speed; a strongly damped contact (r below about 0.5)
rebounds higher than r, as its force, which never pulls, ends before the
spring is relaxed; and bodies on joints move with an effective mass of their
own.

Friction is Coulomb friction regularized below friction_velocity, against the
sliding velocity \f$\underline{u}\f$ at the point:

\f[
\underline{f}_t = -\frac{\mu f \underline{u}}
  {\sqrt{|\underline{u}|^2 + u_0^2}}
\f]

with \f$u_0\f$ the friction_velocity.

*Stick-slip friction.* Regularized Coulomb friction lets a body held by friction
creep: it slides at the speed where the friction it gives equals the load, a
fraction of friction_velocity. A pair of geometries listed in stick_slip has
friction that holds at rest instead, by the bristle model of friction of
Canudas de Wit, Olsson, Astrom and Lischinsky (LuGre, 1995), at each point of
its contact. The bristles at a point are a deflection \f$\underline{z}\f$ in
the tangent plane, with the rate

\f[
\dot{\underline{z}} = \underline{u} - \frac{\sigma_0 |\underline{u}|}{g(|\underline{u}|)} \underline{z},
\quad g(|\underline{u}|) = \mu_c + (\mu_s - \mu_c) e^{-(|\underline{u}| / v_s)^2}
\f]

for the sliding velocity \f$\underline{u}\f$ there, and the friction force on b
is

\f[
\underline{f}_t = -f (\sigma_0 \underline{z} + \sigma_1 \dot{\underline{z}} +
  \sigma_2 \underline{u})
\f]

for the normal force \f$f\f$ at the point: bristle_stiffness \f$\sigma_0\f$ and
bristle_damping \f$\sigma_1\f$ (1/m and s/m, per unit of normal force),
viscous_friction \f$\sigma_2\f$, the friction coefficient \f$\mu_c\f$ of the pair
(the Coulomb friction of the materials), static_friction \f$\mu_s\f$ (at least
\f$\mu_c\f$, default \f$\mu_c\f$) and the Stribeck velocity \f$v_s\f$. At rest
the bristles hold: the deflection settles where \f$\sigma_0 \underline{z}\f$
carries the tangential load, up to \f$\mu_s f\f$, and the body slides only
beyond it, with friction that falls from \f$\mu_s\f$ to \f$\mu_c\f$ as it speeds
up. The deflections of
a pair are internal states of the module: three components for each of a fixed
number of slots (points), filled in the order of the points of the contact
(slot k holds the k-th point of the manifold), each relaxing to zero over
release_time when it is not in contact. They are logged as the output
stick_<a>_<b>, the largest \f$|\underline{z}| \sigma_0 / \mu_s\f$ of the pair
(1 at the limit of holding). The bristles are stiff, relaxing in
\f$g / (\sigma_0 |\underline{u}|)\f$ at the sliding speed
\f$|\underline{u}|\f$, which the variable step integrator has to resolve: a
stiffer \f$\sigma_0\f$ holds closer and takes smaller steps while the body
slides.

Each material may wear by Archard's law: the volume worn from a geometry
grows at

\f[
\dot{q} = k_w \, f |\underline{u}|
\f]

at each of its points of contact, for the normal force \f$f\f$ and the sliding
speed \f$|\underline{u}|\f$ there, with wear_coefficient \f$k_w\f$ of its material (Archard's wear
coefficient over the hardness, volume per normal force and sliding
distance), the worn volume \f$q\f$. The worn volume of each geometry whose material wears is an
internal state of the module, logged as the output wear_<geometry>; it does
not change the shape of the geometry.

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

  /*! \brief the elastic law of a point of CAD contact
   * (collision_geometry::ContactPair::point_laws), \f$f = k d^n\f$ for the
   * modulus \f$e^*\f$ of the pair: of a line (\f$n = 1\f$) for a point carrying
   * a length of a line of contact, Hertz (\f$n = 3/2\f$) otherwise; returns the
   * stiffness linearized at depth, \f$\mathrm{d}f / \mathrm{d}d\f$, for
   * \f$n = 3/2\f$ */
  static double PointLaw(const collision_geometry::ContactPoint& c,
                         double modulus, double depth, double& stiffness,
                         double& exponent);

  /*! \brief resolve the materials of the collision geometry, combine the
   * properties of each pair of materials and exclude jointed bodies */
  virtual void Load() override;
  virtual void Service(const double* y, double* dy) override;
  /*! \brief the volume worn from each collision geometry whose material
   * wears, named wear_\<geometry\> */
  virtual std::vector<std::string> get_output_names() const override;
  virtual void UpdateOutputs(double t, const double* y) override;
  virtual void AppendOutputs(std::vector<double>& values) const override;
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
    int wear = -1; //!< index of its worn volume among the states, -1 if none
  };
  /* a pair with stick-slip friction, resolved from stick_slip */
  struct Bristles {
    std::string name;     //!< <a>_<b>
    double sigma_0 = 0.0, sigma_1 = 0.0, sigma_2 = 0.0;
    double mu_static = 0.0, mu_coulomb = 0.0, velocity = 0.0;
    double release_time = 0.0;
    int points = 0;
    size_t offset = 0; //!< of its first state among the module's
  };

  double Combine(const std::string& rule, double a, double b) const;
  const Geometry& Find(const collision_geometry::Base* geometry);

  static bool registered_;
  parameter::Array<ContactMaterial> materials_;
  parameter::Array<ContactMaterialPair> pairs_;
  parameter::Array<StickSlipPair> stick_slip_;
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
  /*! names of the geometries whose material wears, in the order of their
   * states, and their worn volumes at the last UpdateOutputs */
  std::vector<std::string> wear_names_;
  std::vector<double> worn_;
  /* the pairs with stick-slip friction, and how they are found from the
  pair of collision geometry (in either order) */
  std::vector<Bristles> bristles_;
  std::map<std::pair<const collision_geometry::Base*,
                     const collision_geometry::Base*>,
           int>
    bristle_index_;
  std::vector<double> stick_ratios_; //!< at the last UpdateOutputs
  double GroupMass(int load_index) const;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
