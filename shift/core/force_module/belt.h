#ifndef SHIFT_FORCEMODULE_BELT_H_
#define SHIFT_FORCEMODULE_BELT_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "ksl/screw.h" // IWYU pragma: keep

#include "shift/core/force_module/base.h"

namespace shift {
namespace core {

class Frame;

namespace body {
class Base;
}

namespace force_module {

/*!
@brief An elastic belt around two or more pulleys on bodies

The belt runs in a closed loop around the pulleys in the order given, then
back to the first. Each pulley is a frame on a body: its origin is the centre
of the pulley and its z axis the axis it turns about. The belt lies in the
plane through the centre of the first pulley, normal to its z axis, and the
centres of the other pulleys are projected onto that plane. Looking down
that z axis, the belt travels counterclockwise around the pulleys inside the
loop, so the pulleys must be listed counterclockwise. A negative radius puts
a pulley outside the loop, as an idler running on the back of the belt.

Between pulleys the belt is a straight span along the common tangent of the
two pulleys. A span's tension follows from its stretch \f$e\f$, its length \f$l\f$ less
the unstretched length of the belt between its ends:

\f[
\tau = \frac{k_a \, e}{l - e} + \mathrm{damping} \frac{k_a}{l - e} \dot{e},
\quad \tau \ge 0
\f]

and a span that would be in compression is slack and carries no force; the
change from slack to taut is smoothed over slack_band. The tension pulls the
pulley at each end of the span along the span, at its tangent point. The belt
carries no bending, and its mass is not modelled.

On each pulley the belt is held without slipping, as a toothed belt; the
unstretched length of a span then follows from the turns of the pulleys at
its ends. For a friction belt, friction_coefficient lets the belt slip:
on a pulley of wrap angle \f$\phi\f$, the friction holds tensions with a
ratio up to \f$\exp(\mu \phi)\f$ (the capstan equation), and the belt slides
over the pulley at the speed
\f$(|\tau_{out} - \tau_{in}| - \tau_{slack} (\exp(\mu \phi) - 1)) / \mathrm{slip\_damping}\f$
in the direction of the larger tension once the difference exceeds that.

The module's states are the turn of each pulley relative to the line from
the first pulley's centre to the second's, and, with friction, the length of
belt that has slipped over each pulley; all start at zero.

JSON:
\code
{"type": "belt", "name": "fan_belt",
 "frames": [["crank", "pulley"], ["alternator", "pulley"],
            ["tensioner", "pulley"]],
 "radii": [0.08, 0.035, -0.03],
 "axial_stiffness": 1.5e5, "damping": 1e-3, "pretension": 400,
 "friction_coefficient": [0.5, 0.5, 0.5], "slip_damping": 50}
\endcode

- frames: [body, frame] of each pulley, two or more, counterclockwise about
  the first pulley's z axis.
- radii: the radius of each pulley at the belt's pitch line; negative for a
  pulley outside the loop.
- axial_stiffness: \f$k_a\f$, the product of the modulus and the area of the
  belt, force per unit strain.
- damping (optional, default 0): stiffness-proportional damping of the
  stretch, seconds.
- pretension (optional, default 0): the tension of every span as the model
  starts.
- friction_coefficient (optional): one per pulley; without it the belt does
  not slip.
- slip_damping: with friction_coefficient, the force difference per unit
  slip speed beyond the friction limit, N s/m.
- slack_band (optional, default 1e-6 axial_stiffness): the force over which
  a span changes from slack to taut.
*/
class Belt : public Base {
public:
  Belt()
    : Base() {
    DeclareParameters();
  }
  Belt(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Belt() override {}

  static std::unique_ptr<Belt> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  void UpdateOutputs(double t, const double* y) override;

  size_t get_pulley_count() const {
    return pulleys_.size();
  }
  bool slips() const {
    return !friction_coefficient_.empty();
  }
  /*! \brief of each span k, from pulley k to pulley k + 1 (the last to the
   * first), as of the last Service or UpdateOutputs: the length between
   * its tangent points, the stretch and its rate, and the tension */
  const std::vector<double>& get_span_lengths() const {
    return length_;
  }
  const std::vector<double>& get_stretches() const {
    return stretch_;
  }
  const std::vector<double>& get_stretch_rates() const {
    return stretch_rate_;
  }
  const std::vector<double>& get_tensions() const {
    return tension_;
  }
  /*! \brief of each pulley: the angle the belt wraps around it, and the
   * speed at which the belt slips forward over it */
  const std::vector<double>& get_wrap_angles() const {
    return wrap_;
  }
  const std::vector<double>& get_slip_speeds() const {
    return slip_;
  }

protected:
  virtual void DeclareParameters() override;

private:
  struct Pulley {
    body::Base* body = nullptr;
    const Frame* frame = nullptr;
    int load_index = -2; //!< see Context::GetBodyLoadIndex; -2: not yet
    double radius = 0.0;
    ksl_SE3_t pose = ksl_SE3_identity(); //!< in the fixed frame
    ksl_screw_t velocity = {}; //!< spatial, referred to the fixed origin
  };

  ksl_SE3_t Pose(const Pulley& pulley) const;
  /*! \brief the geometry, stretches, tensions and slip for the module's
   * states, and their rates unless rates is null; with
   * initial, the angles of the spans are taken as they are and kept as the
   * reference for later evaluations */
  void Evaluate(const double* states, double* rates, bool initial = false);
  /*! \brief max(x, 0), made C1 over [0, band] and exact beyond */
  static double Ramp(double x, double band);

  std::vector<std::vector<std::string>> frame_names_;
  std::vector<double> radii_;
  double axial_stiffness_ = 0.0;
  double damping_ = 0.0;
  double pretension_ = 0.0;
  std::vector<double> friction_coefficient_;
  double slip_damping_ = 0.0;
  double slack_band_ = -1.0;

  std::vector<Pulley> pulleys_;
  std::vector<double> angle_0_;  //!< angle of each span's normal at the start
  std::vector<double> offset_;   //!< stretch less the geometric terms
  std::vector<double> angle_;    //!< angle of each span's normal
  std::vector<double> length_, stretch_, stretch_rate_, tension_;
  std::vector<double> spring_tension_; //!< without damping, for the slip
  std::vector<double> wrap_, slip_;
  std::vector<ksl_vec3_t> direction_;   //!< of each span, fixed frame
  std::vector<ksl_vec3_t> start_point_; //!< tangent points, fixed frame
  std::vector<ksl_vec3_t> end_point_;

  static bool registered_;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
