#ifndef SHIFT_CORE_FORCE_MODULE_LEAD_SCREW_H_
#define SHIFT_CORE_FORCE_MODULE_LEAD_SCREW_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "shift/core/force_module/base.h"

namespace shift {
namespace core {

class Frame;
namespace body {
class Base;
}

namespace force_module {

/*!
@brief the thread between a screw and its nut: the force that turns the
rotation of one into the push of the other, with the friction of the thread

The screw and the nut are two bodies, each with a frame whose z axis is the
axis of the thread. The thread keeps them to the motion of a screw: the
screw, turned by the angle \f$\theta\f$ about the axis relative to the nut,
advances along it by \f$\frac{\mathrm{lead}}{2 \pi} \theta\f$, for a
right-handed thread. The module makes
the thread an elastic contact, so that the bodies can be free in the
rotation and the advance (the joint of a model is then a revolute joint and a
prismatic joint in series, or other joints), and the load the thread carries
and the friction on it follow:

- the advance beyond that of the ideal screw is the deflection
  \f$\delta = s - \frac{\mathrm{lead}}{2 \pi} \theta\f$, with \f$s\f$ the
  relative advance along the axis (zero at the start; the thread is unloaded
  then);
- the flank of the thread has the lead angle \f$\lambda\f$,
  \f$\tan\lambda = \mathrm{lead} / (2 \pi r)\f$ with \f$r\f$ the mean_radius,
  along the axis' and the circle's directions, and the half angle \f$\alpha\f$
  of the thread in a plane through the axis (0 for a square thread, 14.5
  degrees for an Acme thread). The deflection moves the flanks together along
  the normal by \f$\delta \cos\alpha \cos\lambda\f$, and the normal force of
  the contact is
  \f[
  n = (k \delta + c \dot{\delta}) \cos\alpha \cos\lambda,
  \f]
  with the stiffness \f$k\f$ and the damping \f$c\f$ of the thread, and its radial
  part (\f$n \sin\alpha\f$) cancelling around the thread. Its axial part
  \f$n \cos\alpha \cos\lambda\f$ acts on the screw along the axis, and its
  circular part gives the torque on it about the axis,
  \f$\mathrm{lead} / 2 \pi\f$ times the axial part: the screw is driven by
  torque and delivers axial force as the ideal screw does;
- friction opposes the sliding of the flank over the other, which is along the
  thread, with the speed
  \f$v = \sqrt{(r \omega)^2 + \dot{s}^2}\f$ for the angular speed
  \f$\omega\f$ and the advance speed \f$\dot{s}\f$ of the screw relative to the
  nut: the friction force is
  \f$\mu |n| v / \sqrt{v^2 + v_s^2}\f$, with \f$v_s\f$ the slip_speed, the smooth
  form of Coulomb friction, along the sliding. Its axial and circular parts
  loads the screw as the thread's sliding does: they add to the axial force
  and torque. The friction is the loss of the screw: the torque to drive it
  exceeds that of the ideal screw.

In steady motion, the torque to raise (drive against) an axial load \f$f\f$ is
the well known

\f[
\tau = f r \, \frac{\tan\lambda + \mu \sec\alpha}{1 - \mu \sec\alpha \tan\lambda},
\f]

and to lower it,
\f$f r \, \frac{\mu \sec\alpha - \tan\lambda}{1 + \mu \sec\alpha \tan\lambda}\f$,
which is negative (the load turns the screw) if
\f$\mu \sec\alpha < \tan\lambda\f$: the screw is not self-locking (Budynas and Nisbett, Shigley's
Mechanical Engineering Design, power screws). The module gives these in
steady motion, as the tests check, and what they leave out: the elastic
deflection of the thread, the damping and the backlash.

The friction is Coulomb friction smoothed at low speed, so a load held on a
self-locking screw creeps slowly (at about slip_speed times the ratio of the
force needed to the friction available, when that is below 1), where a real
screw would stay; stick-slip friction is to be added (see the roadmap). Make
slip_speed small against the speeds of the motion to keep the creep small.

A backlash \f$b\f$ (the axial play in the thread) leaves the deflection free
for \f$|\delta| < b / 2\f$, and the thread loads only beyond it, smoothly. The
thrust collar's friction (the friction of the load's bearing on the screw) is not
included; add it with the friction of the joint carrying the screw.

JSON:
\code
{"type": "lead_screw", "name": "jack",
 "screw": ["screw", "axis"], "nut": ["fixed", "nut_axis"],
 "lead": 0.005, "mean_radius": 0.012, "thread_angle": 0.253,
 "friction": 0.15, "stiffness": 1e8, "damping": 1e4}
\endcode

- screw, nut: [body, frame] of each; the z axis of the nut's frame is the axis
  of the thread, and the frames are on it (the body "fixed" may be used).
- lead: the advance of the screw per turn (m for SI).
- mean_radius: the radius at which the thread's flank carries the load.
- thread_angle (optional, default 0, radians): the half angle alpha of the
  thread.
- friction: the coefficient of friction on the thread (mu).
- stiffness: the stiffness of the thread's contact along its normal (N/m);
  damping (optional, default 0): its damping (N s/m).
- backlash (optional, default 0): the axial play.
- slip_speed (optional, default 1e-3 m/s): the speed below which the friction
  goes smoothly to zero.
- left_handed (optional, default false).

Outputs: axial_force (on the screw along the axis, N), torque (on the screw
about the axis), deflection, normal_force, friction_force (its magnitude,
mu |N| smoothed) and turns (the relative rotation of the screw, in turns).
*/
class LeadScrew : public Base {
public:
  LeadScrew()
    : Base() {
    Initialize();
  }
  LeadScrew(std::string name)
    : Base(name) {
    Initialize();
  }
  virtual ~LeadScrew() override {}

  static std::unique_ptr<LeadScrew> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  std::vector<std::string> get_output_names() const override;
  void UpdateOutputs(double t, const double* y) override;
  void AppendOutputs(std::vector<double>& values) const override;

  /*! \brief the lead angle of the thread,
   * \f$\arctan(\mathrm{lead} / (2 \pi r))\f$ */
  double get_lead_angle() const;
  /*! \brief whether the thread holds a load when it is not driven,
   * \f$\mu \sec\alpha \ge \tan\lambda\f$ */
  bool is_self_locking() const;

  double get_axial_force() const {
    return axial_force_;
  }
  double get_torque() const {
    return torque_;
  }
  double get_deflection() const {
    return deflection_;
  }
  double get_normal_force() const {
    return normal_force_;
  }
  double get_friction_force() const {
    return friction_force_;
  }
  /*! \brief the rotation of the screw relative to the nut, in turns */
  double get_turns() const {
    return turns_;
  }

protected:
  void DeclareParameters() override;

private:
  void Initialize() {
    DeclareParameters();
  }

  struct Attachment {
    body::Base* body = nullptr;
    const Frame* frame = nullptr;
    int load_index = -2; //!< see Context::GetBodyLoadIndex; -2: not yet
  };

  ksl_SE3_t Pose(const Attachment& attachment) const;
  Attachment Resolve(const std::vector<std::string>& names,
                     const std::string& what) const;
  void Compute(const double* y, double* dy, bool apply);

  static bool registered_;

  std::vector<std::string> screw_names_, nut_names_;
  double lead_ = 0.0;
  double mean_radius_ = 0.0;
  double thread_angle_ = 0.0;
  double friction_ = 0.0;
  double stiffness_ = 0.0;
  double damping_ = 0.0;
  double backlash_ = 0.0;
  double slip_speed_ = 1e-3;
  bool left_handed_ = false;

  Attachment screw_, nut_;
  double advance_0_ = 0.0; //!< relative advance along the axis at the start
  double axial_force_ = 0.0, torque_ = 0.0, deflection_ = 0.0;
  double normal_force_ = 0.0, friction_force_ = 0.0, turns_ = 0.0;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
