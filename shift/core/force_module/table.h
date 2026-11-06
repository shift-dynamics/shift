#ifndef SHIFT_FORCEMODULE_TABLE_H_
#define SHIFT_FORCEMODULE_TABLE_H_

#include <string>
#include <vector>

namespace shift {
namespace core {
namespace force_module {

/*!
@brief A function of one variable given by a table of points, y(x)

Between the points the function is a cubic in each interval whose slopes at
the points are chosen so that it never overshoots the data: where the data
rise, the curve rises, and where they are level, it is level (the monotone
cubic of Fritsch and Carlson, 1980). The function and its slope are
continuous, so a variable step integrator sees no jumps. Beyond the ends it
holds the end values (Extrapolation::kHold, for inputs that are functions of
time) or continues with the end slopes (Extrapolation::kLinear, for curves
such as force against displacement).
*/
class Table {
public:
  enum class Extrapolation { kHold, kLinear };

  Table() {}
  /*! \brief the table of the points (x[k], y[k]); x must increase */
  Table(const std::vector<double>& x, const std::vector<double>& y,
        Extrapolation extrapolation);
  /*! \brief the table from the pairs [x0, y0, x1, y1, ...]; what names the
   * table in error messages */
  static Table FromPairs(const std::vector<double>& pairs,
                         Extrapolation extrapolation, const std::string& what);

  bool empty() const {
    return x_.empty();
  }
  /*! \brief y(x), and its slope dy/dx if slope is not null */
  double Evaluate(double x, double* slope = nullptr) const;

private:
  std::vector<double> x_, y_, m_; //!< points and slopes at the points
  Extrapolation extrapolation_ = Extrapolation::kHold;
};

/*! \brief the ramp of the smooth edge of a stop and its slope: 0 below
 * \f$-b/2\f$, \f$u\f$ above \f$b/2\f$, with a parabola between */
inline void Ramp(double u, double b, double& value, double& slope) {
  if(u <= -0.5 * b) {
    value = slope = 0.0;
  } else if(u >= 0.5 * b) {
    value = u;
    slope = 1.0;
  } else {
    const double a = u + 0.5 * b;
    value = 0.5 * a * a / b;
    slope = a / b;
  }
}

} // namespace force_module
} // namespace core
} // namespace shift

#endif
