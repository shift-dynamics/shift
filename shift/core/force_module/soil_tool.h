#ifndef SHIFT_FORCEMODULE_SOIL_TOOL_H_
#define SHIFT_FORCEMODULE_SOIL_TOOL_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "ksl/vec3.h"

#include "shift/core/force_module/base.h"
#include "shift/core/force_module/soil.h"

namespace shift {
namespace core {

namespace body {
class Base;
}

namespace force_module {

/*!
@brief the force of the soil on a wide blade cutting it, with the pile the
blade pushes: Reece's fundamental equation of earth moving, with McKyes' factors

The blade is a flat rectangle on a body: its cutting edge through the point tip
along edge, of the given width, rising along face for its height, and working
towards front. All are in the body's coordinates. The ground is the level plane
\f$z = \mathrm{ground}\f$ (z down), the soil below it.

The force per width on a blade cutting to the depth \f$d\f$ is

\f[
f = w d^2 n_w + c d n_c + c_a d n_a + q n_q
\f]

with the unit_weight \f$w\f$, the cohesion \f$c\f$, the adhesion \f$c_a\f$ of the soil
to the blade and the weight \f$q\f$ per width of the pile on the soil in front.
A wedge of soil fails along a plane at the angle \f$\beta\f$ to the ground. With the
blade at the angle \f$\rho\f$ to the ground (90 degrees is vertical, below that its
top trails its edge), the friction angle \f$\phi\f$ of the soil and \f$\delta\f$ (the
wall_friction of the soil) between soil and blade, and
\f$s = \sin(\delta + \rho + \phi + \beta)\f$:

\f[
n_w = \frac{(\cot\rho + \cot\beta) \sin(\phi + \beta)}{2 s}, \quad
n_c = \frac{\cos\phi}{s \sin\beta}, \quad
n_a = \frac{-\cos(\rho + \phi + \beta)}{s \sin\rho}, \quad
n_q = \frac{\sin(\phi + \beta)}{s}
\f]

\f$\beta\f$ is the angle of least \f$n_w\f$, found at each evaluation. For a vertical
blade \f$2 n_w\f$ is the passive coefficient of Coulomb along the line of the
thrust (Rankine's without friction), and \f$n_c\f$ is \f$2 \sqrt{\kappa_p}\f$ at
\f$\delta = 0\f$: see SoilProperties. \f$\rho\f$ is limited below
\f$\pi - \delta - \phi - 0.12\f$, as the wedge has no solution past
\f$\delta + \rho + \phi + \beta = \pi\f$.

The force on the blade, of the width \f$b\f$, is

\f[
m b f \left(-\sin(\rho + \delta) \, \underline{e} -
  \cos(\rho + \delta) \, \underline{u}\right)
\f]

with \f$\underline{e}\f$ the level direction of front and \f$\underline{u}\f$ up: the
draft against the motion, and a pull into the soil for a blade below
\f$90^\circ - \delta\f$. It acts on the face a third of the depth above the edge.
It builds up with the speed \f$v\f$ of the edge along \f$\underline{e}\f$,
\f$m = \max(0, v) / \sqrt{v^2 + v_s^2}\f$ with the slip_speed \f$v_s\f$, so that a
blade backing up or at rest has none.

The pile has the area \f$a\f$ (per width), a state. It grows with the soil cut and
spills over the blade and past its ends beyond the area of a heap of the
blade's height at the angle of repose \f$\phi\f$, \f$a_m = h^2 / (2 \tan\phi)\f$,
and is left behind when the blade backs up or lifts:

\f[
\dot{a} = m v d - \max(0, a - a_m) / t_s - (1 - m) a / t_s
\f]

with the spill_time \f$t_s\f$, and \f$q = w a\f$.

Not modelled: the shape the blade leaves in the ground, which stays level (see
the deformable terrain on the roadmap); the inertia of the soil, the soil moving
along an angled blade, the penetration of a blade pressed down at rest, and
narrow tines and rippers, whose soil also fails to the sides.

Outputs: depth, rake, failure_angle, pile, draft (against front) and lift (up).

JSON (SI units):
\code
{"type": "soil_tool", "name": "blade", "soil": {"name": "sandy_loam",
 "wall_friction": 0.25}, "body": "dozer", "tip": [2.5, 0, 0.6],
 "face": [0, 0, -1], "edge": [0, 1, 0], "front": [1, 0, 0], "width": 3.2,
 "height": 1.1}
\endcode

References: A.R. Reece, Proc. Inst. Mech. Eng. 179(6), 16-22 (1964); E. McKyes,
Soil Cutting and Tillage (1985); W.J. Wagner, arXiv:2505.19330 (2025).
*/
class SoilTool : public Base {
public:
  /*! \brief the factors of the equation, and the failure angle */
  struct Factors {
    double weight = 0.0, cohesion = 0.0, adhesion = 0.0, surcharge = 0.0;
    double failure_angle = 0.0;
  };

  SoilTool()
    : Base() {
    DeclareParameters();
  }
  SoilTool(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~SoilTool() override {}

  static std::unique_ptr<SoilTool> Create();
  static bool is_registered() {
    return registered_;
  }

  /*! \brief the factors for the blade angle rake, and the friction angles of
   * the soil and between soil and blade */
  static Factors ComputeFactors(double rake, double friction_angle,
                                double wall_friction);

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  std::vector<std::string> get_output_names() const override;
  void UpdateOutputs(double t, const double* y) override;
  void AppendOutputs(std::vector<double>& values) const override;

  double get_depth() const {
    return depth_;
  }
  double get_draft() const {
    return draft_;
  }
  double get_lift() const {
    return lift_;
  }
  double get_pile() const {
    return pile_;
  }

protected:
  void DeclareParameters() override;

private:
  void Evaluate(const double* y, double* dy, bool apply);

  static bool registered_;

  Soil soil_;
  std::string body_name_;
  ksl_vec3_t tip_ = {{0.0, 0.0, 0.0}};
  ksl_vec3_t face_ = {{0.0, 0.0, -1.0}};
  ksl_vec3_t edge_ = {{0.0, 1.0, 0.0}};
  ksl_vec3_t front_ = {{1.0, 0.0, 0.0}};
  double width_ = 0.0, height_ = 0.0;
  double ground_ = 0.0;
  double adhesion_ = 0.0;
  double slip_speed_ = 0.01;
  double spill_time_ = 1.0;

  SoilProperties soil_properties_;
  body::Base* body_ = nullptr;
  int load_index_ = -2;
  double depth_ = 0.0, rake_ = 0.0, failure_angle_ = 0.0, pile_ = 0.0;
  double draft_ = 0.0, lift_ = 0.0;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
