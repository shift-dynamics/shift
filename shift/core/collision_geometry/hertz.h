#ifndef SHIFT_CORE_COLLISION_GEOMETRY_HERTZ_H_
#define SHIFT_CORE_COLLISION_GEOMETRY_HERTZ_H_

namespace shift {
namespace core {
namespace collision_geometry {

/*!
@brief Hertz contact of two surfaces touching at a point, whose gap near the
point is h = A x^2 + B y^2 (B >= A > 0, the half curvatures of the gap)

The contact area is an ellipse, and the force at depth d is

  F = 4/3 E* sqrt(1 / (A + B)) g(B / A) d^(3/2)

with g = 1 for a circle (A = B, e.g. two spheres). The ellipse's
eccentricity e follows from B / A by Hertz's equations, with the complete
elliptic integrals K(e) and E(e) (Johnson, Contact Mechanics, 1985, 4.2):

  B / A = (E / (1 - e^2) - K) / (K - E),
  F = 2 pi E* / (3 K) sqrt((K - E) / (K A e^2)) d^(3/2)

The factor grows without bound as B / A does, towards a line of contact,
whose law differs (see force_module::Contact); ratio is limited to 1e6.
If elongation is given, it is set to the ratio a / b of the ellipse's axes.
*/
double HertzEllipseFactor(double ratio, double* elongation = nullptr);

/*!
@brief complete elliptic integrals of the first and second kind, K and E,
of parameter m = e^2, given 1 - m (accurate as m tends to 1), by the
arithmetic-geometric mean; also K - E, accurate as m tends to 0
*/
void CompleteEllipticIntegrals(double m, double complement, double& k,
                               double& e, double& k_minus_e);

} // namespace collision_geometry
} // namespace core
} // namespace shift

#endif // SHIFT_CORE_COLLISION_GEOMETRY_HERTZ_H_
