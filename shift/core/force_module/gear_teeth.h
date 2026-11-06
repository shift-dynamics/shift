#ifndef SHIFT_CORE_FORCE_MODULE_GEAR_TEETH_H_
#define SHIFT_CORE_FORCE_MODULE_GEAR_TEETH_H_

#include "shift/core/parameter/list.h"

namespace shift {
namespace core {
namespace force_module {

/*!
@brief the stiffness of a pair of involute gears, from the shape of their teeth

The module computes the stiffness of a pair of teeth at each point of the path
of contact, and the mesh stiffness from the pairs in contact at each position
in the mesh, for GearMesh.

*The path of contact.* With the pitch radii \f$r_1\f$, \f$r_2\f$ and the
transverse pressure angle \f$\phi\f$, the base radii are
\f$r_{bk} = r_k \cos\phi\f$. A point of contact lies on the line of action, at the
distance \f$\rho_k\f$ from where the line touches the base circle of gear
\f$k\f$, and for the center distance \f$d\f$ the distances are related by
\f$\rho_1 + \rho_2 = d \sin\phi\f$ for external gears and
\f$\rho_2 - \rho_1 = d \sin\phi\f$ for internal gears. Contact begins and ends
at the tip circles, of radius \f$r_{ak}\f$, where
\f$\rho_k = \sqrt{r_{ak}^2 - r_{bk}^2}\f$. The length of this path over the base
pitch \f$2 \pi r_{b1} / z_1\f$ is the contact ratio.

*A pair of teeth.* The compliance of a pair is the sum of the Hertzian
compliance of the contact and the bending, shear, axial and fillet-foundation
compliances of each tooth (the potential energy method of Yang and Lin, 1987,
with the shear added by Tian, 2004):

\f[
\frac{1}{k} = \frac{1}{k_h} + \sum_{i = 1}^{2} \left(\frac{1}{k_{bi}} +
  \frac{1}{k_{si}} + \frac{1}{k_{ai}} + \frac{1}{k_{fi}}\right)
\f]

With Young's modulus \f$e_m\f$, Poisson's ratio \f$\nu\f$, the face width \f$w\f$,
the angle \f$\alpha_2 = \pi / 2 z + \tan\phi - \phi\f$ of half a tooth at its base
circle, and \f$\alpha_1 = \rho / r_b - \alpha_2\f$ the angle of the load to
the perpendicular of the tooth's center line, with
\f$g(\alpha) = \sin\alpha + (\alpha_2 - \alpha) \cos\alpha\f$:

\f[
\begin{aligned}
\frac{1}{k_h} &= \frac{4 (1 - \nu^2)}{\pi e_m w} \\
\frac{1}{k_b} &= \int_{-\alpha_1}^{\alpha_r}
  \frac{3 \left(1 + \cos\alpha_1 \left((\alpha_2 - \alpha) \sin\alpha -
  \cos\alpha\right)\right)^2 (\alpha_2 - \alpha) \cos\alpha}
  {2 e_m w \, g(\alpha)^3} \, \mathrm{d}\alpha \\
\frac{1}{k_s} &= \int_{-\alpha_1}^{\alpha_r}
  \frac{6 (1 + \nu) (\alpha_2 - \alpha) \cos\alpha \cos^2\alpha_1}
  {5 e_m w \, g(\alpha)} \, \mathrm{d}\alpha \\
\frac{1}{k_a} &= \int_{-\alpha_1}^{\alpha_r}
  \frac{(\alpha_2 - \alpha) \cos\alpha \sin^2\alpha_1}
  {2 e_m w \, g(\alpha)} \, \mathrm{d}\alpha
\end{aligned}
\f]

The tooth is a cantilever that starts at the base circle (\f$\alpha_r = \alpha_2\f$),
or at the root circle if that is the larger of the two. The gear's body gives
under the tooth (Sainsot, Velex and Duverger, 2004):

\f[
\frac{1}{k_f} = \frac{\cos^2\alpha_1}{e_m w} \left(
  l^* \left(\frac{u_f}{s_f}\right)^2 + m^* \frac{u_f}{s_f} +
  p^* (1 + q^* \tan^2\alpha_1)\right)
\f]

with \f$u_f\f$ the distance along the center line from the root circle to the
line of the load, \f$s_f\f$ the thickness of the tooth at the root, and
\f$l^*\f$, \f$m^*\f$, \f$p^*\f$, \f$q^*\f$ their polynomial fits in the ratio
of the root radius to the hub radius, from 1.4 to 7, and the half angle of the
tooth at the root.

Not modelled: the corrections of the profile, the shift of the teeth, the
growth of the contact under load, and the coupling of neighboring teeth through
the gear's body. A tooth of a ring gear is taken as that of an external gear of the same
number of teeth, loaded at the same height.

JSON, the "tooth_geometry" of a GearMesh:
\code
"tooth_geometry": {"youngs_modulus": 2.06e11, "poisson_ratio": 0.3}
\endcode
*/
class GearTeeth : public parameter::List {
public:
  GearTeeth() {
    DeclareParameters();
  }
  virtual ~GearTeeth() {}

  double youngs_modulus = 0.0; //!< 0: the stiffness is not computed
  double poisson_ratio = 0.3;
  double addendum = 1.0;   //!< tip above the pitch circle, in modules
  double dedendum = 1.25;  //!< root below the pitch circle, in modules
  double hub_ratio = 0.5;  //!< hub radius over root radius
  int points = 200;        //!< of the table of the stiffness over a mesh

  /*! \brief sets the gears: pitch radii, teeth, the transverse pressure angle,
   * the module of the addendum and dedendum, and whether gear 2 is a ring;
   * throws if the teeth do not mesh */
  void Configure(double radius_1, double radius_2, int teeth_1, int teeth_2,
                 double pressure_angle, double module, bool internal);

  double get_contact_ratio() const {
    return contact_ratio_;
  }

  /*! \brief the stiffness of a pair of teeth of the face width at the fraction
   * s of a base pitch along the path of contact, from 0 to the contact ratio */
  double PairStiffness(double s, double width) const;

  /*! \brief the stiffness of the mesh of the face width at the position x, the
   * sum over the pairs in contact; x = 0 is the middle of the part of the mesh
   * with the most pairs, as in GearMesh */
  double MeshStiffness(double x, double width) const;

protected:
  void DeclareParameters() override;

private:
  struct Gear {
    double base = 0.0, root = 0.0;
    int teeth = 0;
  };

  /*! \brief the compliance of a tooth of unit width and unit modulus, loaded at
   * the distance rho along the line of action */
  double ToothCompliance(const Gear& gear, double rho) const;

  Gear gear_1_, gear_2_;
  bool internal_ = false;
  double phi_ = 0.0;
  double rho_start_ = 0.0;      //!< rho_1 where contact begins
  double base_pitch_ = 0.0;
  double contact_ratio_ = 0.0;
  double rho_sum_ = 0.0;        //!< rho_1 + rho_2 of external gears
  double ring_tip_ = 0.0;       //!< rho at the tip of a ring's tooth
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
