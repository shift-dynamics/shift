#ifndef SHIFT_FORCEMODULE_SPRING_DAMPER_H_
#define SHIFT_FORCEMODULE_SPRING_DAMPER_H_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "ksl/vec3.h"

#include "shift/core/force_module/base.h"
#include "shift/core/force_module/table.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {

class Frame;

namespace body {
class Base;
}

namespace force_module {

/*!
@brief Bouc-Wen hysteresis: a force that depends on the history of a
displacement, through a state

The force is \f$f_y z\f$ with the yield_force \f$f_y\f$ and the dimensionless
state \f$z\f$, which follows the displacement \f$x\f$:

\f[
\dot{z} = \frac{\dot{x}}{x_y} \left(1 - |z|^n \left(\beta + \gamma \,
  \mathrm{sign}(\dot{x} z)\right)\right)
\f]

with the yield_displacement \f$x_y\f$, the exponent \f$n\f$ and the shape
parameters \f$\beta\f$ and \f$\gamma\f$. The force rises with the slope
\f$f_y / x_y\f$ and, with \f$\beta + \gamma = 1\f$, levels off at \f$f_y\f$. The
rate of \f$z\f$ is proportional to \f$\dot{x}\f$, so the loop does not depend on how
fast it is run.

Reference: Y.K. Wen, J. Engineering Mechanics 102(2), 249-263 (1976).
*/
class BoucWen : public parameter::List {
public:
  BoucWen() {
    DeclareParameters();
  }
  virtual ~BoucWen() {}

  bool is_used() const {
    return yield_force > 0.0;
  }
  /*! \brief throws if the parameters are out of range */
  void Check(const std::string& owner) const;
  /*! \brief the rate of the state z at the rate of the displacement */
  double Rate(double z, double displacement_rate) const;

  double yield_force = 0.0;
  double yield_displacement = 0.0;
  double exponent = 1.0;
  double beta = 0.5;
  double gamma = 0.5;

protected:
  void DeclareParameters() override;
};

/*!
@brief A spring and damper between two frames, along the line that joins their
origins or about an axis, with curves from measurements, stops and hysteresis

Along the line, for a coil spring or a shock absorber between a chassis and an
axle: with \f$\underline{p}_i\f$ and \f$\underline{p}_j\f$ the origins of the two
frames, the length is \f$l = |\underline{p}_j - \underline{p}_i|\f$, the deflection
\f$x = l - l_0\f$ from the free_length \f$l_0\f$, and its rate
\f$v = \dot{l}\f$. The force \f$f\f$, positive in tension, pulls the two points
together: \f$-f \underline{n}\f$ on frame j and \f$f \underline{n}\f$ on frame i, with
\f$\underline{n} = (\underline{p}_j - \underline{p}_i) / l\f$, the line turning with
the bodies. It is

\f[
f = w \left(s(x_e) + d(v) + f_y z\right)
\f]

The spring \f$s\f$ is \f$k x_e + f_0\f$, with the stiffness \f$k\f$ and the preload
\f$f_0\f$, or a force_curve of points, and the damper \f$d\f$ is \f$c v\f$ or a
damping_curve. A curve has its own sign on each side of zero, so that it can
differ in compression and rebound. The curves are monotone cubics
(force_module::Table), continuing with the slope of their ends.

With a gap \f$g\f$ or acts "compression" or "tension", the spring is a stop: acts
"compression" acts on \f$x < -g\f$ only, "tension" on \f$x > g\f$ only, and "both" on
both, with nothing between. Then \f$x_e\f$ is the deflection past the edge, and
\f$w\f$ goes from 0 to 1 over the width smoothing around it, as does the
slope of \f$x_e\f$, so that the integrator sees no jump: in terms of the
distance \f$u\f$ past the edge, \f$x_e = \rho(u)\f$ and \f$w = \rho'(u)\f$ with
\f$\rho(u) = (u + b/2)^2 / (2 b)\f$ for \f$|u| < b/2\f$, \f$b\f$ the smoothing, 0 below
that and \f$u\f$ above. The force of a stop only ever pushes it open: it is clipped at zero as
a fast rebound would make the damper pull. Without a stop, \f$x_e = x\f$, \f$w = 1\f$.

With a hysteresis block, \f$f_y z\f$ is a Bouc-Wen hysteresis of \f$x\f$ (see BoucWen),
one state.

With unloading_stiffness \f$k_u\f$ the spring crushes, as a protective structure
does in a rollover: in compression it follows \f$s\f$ to the deepest
compression \f$p\f$ reached, a state with
\f$\dot{p} = \max(0, -x_e - p) / t_c\f$ for the crush_time \f$t_c\f$, and from there it
comes back along the line of slope \f$k_u\f$, down to zero. The structure stays
crushed, and the output absorbed is the work the force has done on it, a state.

About an axis, for a torsion spring: with axis (in frame i's coordinates), the
same law gives the moment about the axis, \f$-f \underline{a}\f$ on frame j and
\f$f \underline{a}\f$ on frame i. \f$x\f$ is the angle turned from free_angle, a state, the
integral of the relative angular velocity about the axis, so it counts
full turns, with \f$v\f$ that velocity.

Outputs: length (or angle), rate, force (or moment), positive in tension; with
unloading_stiffness also crush and absorbed.

JSON (SI units):
\code
{"type": "spring_damper", "name": "front_shock",
 "frames": [["chassis", "shock_top"], ["axle", "shock_bottom"]],
 "free_length": 0.45, "stiffness": 30000, "preload": -500,
 "damping_curve": [-1.0, -3000, -0.1, -800, 0, 0, 0.1, 1500, 1.0, 6000]}
{"type": "spring_damper", "name": "bump_stop",
 "frames": [["chassis", "stop"], ["axle", "stop"]], "free_length": 0.3,
 "acts": "compression", "gap": 0.08, "stiffness": 200000, "damping": 2000,
 "smoothing": 0.002}
{"type": "spring_damper", "name": "hinge_spring", "axis": [0, 1, 0],
 "frames": [["door_frame", "hinge"], ["door", "hinge"]], "stiffness": 40,
 "hysteresis": {"yield_force": 3, "yield_displacement": 0.02}}
\endcode
*/
class SpringDamper : public Base {
public:
  SpringDamper()
    : Base() {
    DeclareParameters();
  }
  SpringDamper(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~SpringDamper() override {}

  static std::unique_ptr<SpringDamper> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  void UpdateOutputs(double t, const double* y) override;

  /*! \brief the length (angle), its rate and the force (moment), positive in
   * tension, as of the last Service or UpdateOutputs */
  double get_length() const {
    return length_;
  }
  double get_rate() const {
    return rate_;
  }
  double get_force() const {
    return force_;
  }

  /*! \brief the law: the force at the deflection x and its rate v, with the
   * hysteresis state z and the deepest compression crush */
  double Force(double x, double v, double z, double crush = 0.0) const;

  double get_crush() const {
    return crush_;
  }
  double get_absorbed() const {
    return absorbed_;
  }

protected:
  void DeclareParameters() override;

private:
  struct Attachment {
    body::Base* body = nullptr;
    const Frame* frame = nullptr;
    int load_index = -2; //!< see Context::GetBodyLoadIndex; -2: not yet
  };

  ksl_SE3_t Pose(const Attachment& a) const;
  /*! \brief the length, rate and force for the state y, and the rates of
   * the states if dy is not null */
  void Compute(const double* y, double* dy);
  void Apply();

  static bool registered_;

  std::vector<std::vector<std::string>> frame_names_;
  ksl_vec3_t axis_ = {{0.0, 0.0, 0.0}};
  double free_length_ = std::numeric_limits<double>::quiet_NaN();
  double free_angle_ = 0.0;
  double stiffness_ = 0.0;
  double preload_ = 0.0;
  double damping_ = 0.0;
  std::vector<double> force_curve_values_;
  std::vector<double> damping_curve_values_;
  std::string acts_ = "both";
  double gap_ = 0.0;
  double smoothing_ = 1e-3;
  double unloading_stiffness_ = 0.0;
  double crush_time_ = 1e-3;
  BoucWen hysteresis_;

  Attachment attachments_[2];
  Table force_curve_, damping_curve_;
  bool torsion_ = false;
  bool stop_ = false;
  bool pushes_ = true, pulls_ = true; //!< which sides of the stop act
  double free_ = 0.0;                 //!< l_0 or the free angle
  ksl_SE3_t poses_[2];
  ksl_vec3_t direction_ = {{0.0, 0.0, 0.0}}; //!< n or a
  double length_ = 0.0, rate_ = 0.0, force_ = 0.0;
  double crush_ = 0.0, absorbed_ = 0.0;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
