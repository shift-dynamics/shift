#ifndef SHIFT_CORE_COLLISION_GEOMETRY_HERTZ_H_
#define SHIFT_CORE_COLLISION_GEOMETRY_HERTZ_H_

namespace shift {
namespace core {
namespace collision_geometry {

/*!
@brief Hertz contact of two surfaces touching at a point, whose gap near the
point is \f$h = a x^2 + b y^2\f$ (\f$b \ge a > 0\f$, the half curvatures of
the gap)

The contact area is an ellipse, and the force at depth \f$d\f$ is

\f[
f = \frac{4}{3} e^* \sqrt{\frac{1}{a + b}} \, g(b / a) \, d^{3/2}
\f]

with \f$g = 1\f$ for a circle (\f$a = b\f$, e.g. two spheres). The ellipse's
eccentricity \f$\epsilon\f$ follows from \f$b / a\f$ by Hertz's equations, with the
complete elliptic integrals \f$k_1(\epsilon)\f$ and \f$k_2(\epsilon)\f$ (Johnson, Contact
Mechanics, 1985, 4.2):

\f[
\begin{aligned}
\frac{b}{a} &= \frac{k_2 / (1 - \epsilon^2) - k_1}{k_1 - k_2} \\
f &= \frac{2 \pi e^*}{3 k_1} \sqrt{\frac{k_1 - k_2}{k_1 a \epsilon^2}} \, d^{3/2}
\end{aligned}
\f]

The factor grows without bound as \f$b / a\f$ does, towards a line of contact,
whose law differs (see force_module::Contact); ratio is limited to 1e6.
If elongation is given, it is set to the ratio of the ellipse's axes.
*/
double HertzEllipseFactor(double ratio, double* elongation = nullptr);

/*!
@brief complete elliptic integrals of the first and second kind, \f$k_1\f$ and \f$k_2\f$,
of parameter \f$m = \epsilon^2\f$, given \f$1 - m\f$ (accurate as \f$m\f$ tends to
1), by the arithmetic-geometric mean; also \f$k_1 - k_2\f$, accurate as \f$m\f$
tends to 0
*/
void CompleteEllipticIntegrals(double m, double complement, double& k,
                               double& e, double& k_minus_e);

} // namespace collision_geometry
} // namespace core
} // namespace shift

#endif // SHIFT_CORE_COLLISION_GEOMETRY_HERTZ_H_
