#ifndef SHIFT_CORE_CAD_EVALUATE_H_
#define SHIFT_CORE_CAD_EVALUATE_H_

#include <array>

#include "shift/core/cad/brep.h"

namespace shift {
namespace core {
namespace cad {

/*!
@brief point of a curve at parameter t, and optionally its first derivative
with respect to t; a NURBS curve is evaluated within its domain (t is
clamped to it)
*/
template <int D>
std::array<double, D> CurvePoint(const Curve<D>& curve, double t,
                                 std::array<double, D>* derivative = nullptr);

/*!
@brief parameters over which a curve is defined: a NURBS curve's knot
domain, [0, 2 pi) for a conic (periodic), and none for a line
*/
struct Interval {
  double lo = 0.0;
  double hi = 0.0;
  bool bounded = false;
  bool periodic = false;
};

template <int D>
Interval CurveDomain(const Curve<D>& curve);

/*!
@brief point of a surface at (u, v), and optionally its derivatives with
respect to u and v (see Surface for the parameterizations)
*/
Vec3 SurfacePoint(const Model& model, const Surface& surface, double u,
                  double v, Vec3* du = nullptr, Vec3* dv = nullptr);

/*!
@brief domains of a surface's parameters u and v: periodic ones (angles of
cylinders, cones, spheres, tori and revolutions, of period 2 pi), bounded
ones (NURBS knot domains, the latitude of a sphere) and unbounded ones
*/
std::array<Interval, 2> SurfaceDomain(const Model& model,
                                      const Surface& surface);

/*!
@brief parameter of the point of a curve closest to p, near guess if given
(on a conic, t is in [0, 2 pi) without a guess and nearest the guess with
one). Returns false if no closest point could be found.
*/
bool ProjectToCurve(const Curve3& curve, const Vec3& p, double& t,
                    const double* guess = nullptr);

/*!
@brief parameters (u, v) of the point of a surface closest to p, near guess
if given: periodic parameters are taken nearest the guess, and where the
surface is singular (the apex of a cone, the poles of a sphere) u is that of
the guess. Returns false if no closest point could be found.
*/
bool ProjectToSurface(const Model& model, const Surface& surface,
                      const Vec3& p, Vec2& uv, const Vec2* guess = nullptr);

/*! @brief distance between points */
double Distance(const Vec3& a, const Vec3& b);

} // namespace cad
} // namespace core
} // namespace shift

#endif
