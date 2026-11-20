#ifndef SHIFT_CONTROLLER_DRIVER_H_
#define SHIFT_CONTROLLER_DRIVER_H_

#include <memory>
#include <string>
#include <vector>

#include <ksl/SE3.h>
#include <ksl/vec3.h>

#include "shift/core/controller/base.h"

namespace shift {
namespace core {

class Frame;
namespace body {
class Base;
}

namespace controller {

/*!
@brief drives a vehicle along a path: steering by pure pursuit, and the
throttle, the brakes and the gear for a speed that the path allows

The path is a line through points \f$[x_0, y_0, x_1, y_1, \ldots]\f$ in the
fixed frame (closed back to the first point if closed). The vehicle is the
frame body_frame_pair, at the middle of the rear axle with its x axis
forward and z down. At each evaluation the driver finds the point of the path
nearest the frame, its station \f$s\f$ (the distance along the path), and the
goal point a look-ahead distance further along it,

\f[
l = l_0 + t_l \, \max(v, 0),
\f]

with \f$v\f$ the speed of the frame along its x axis, \f$l_0\f$
lookahead_distance and \f$t_l\f$ lookahead_time. Pure pursuit steers on the
arc from the frame through the goal point (R. C. Coulter,
Implementation of the Pure Pursuit Path Tracking Algorithm, CMU-RI-TR-92-01,
1992): with the goal at
\f$x_g, y_g\f$ in the coordinates of the frame and the wheelbase \f$w\f$, the
angle of the front wheels of a bicycle model is

\f[
\delta = \arctan \frac{2 \, w \, y_g}{x_g^2 + y_g^2},
\f]

positive about z, so to the right, within \f$\pm\f$ max_steering_angle.

The target speed is the speed profile of the path at the goal point, so that
the driver brakes before a bend: at each point of the path the speed that
keeps the lateral acceleration \f$v^2 \kappa\f$ within
lateral_acceleration, with \f$\kappa\f$ the curvature of the circle through
the path a wheelbase before and after the point, at most speed; then reduced
so that the vehicle reaches each point from the one before at no more than
acceleration, and slows to the one after at no more than deceleration. An
open path ends at rest. From the error \f$e = v_t - v\f$ of the speed, the
throttle is \f$k_t e\f$ and the brake \f$-k_b e\f$, each from 0 to 1, with
\f$k_t\f$ throttle_gain and \f$k_b\f$ brake_gain; where the target is 0, at
the end of an open path, the brake is full. The gear is 1 plus the number of
shift_speeds below the speed.

The driver sets nothing itself: its outputs are read by map controllers
after it in the model file, which set the inputs of the vehicle through a
curve: the steering angle to the target of the steering rack's controller,
by the steering ratio; the throttle to the engine; the brake to the
pressures of the brakes; the gear to the transmission.

JSON:
\code
{"type": "driver", "name": "driver",
 "body_frame_pair": ["chassis", "rear_axle"], "wheelbase": 2.48,
 "path": [0, 0, 20, 0, 40, 5, 60, 15], "closed": false,
 "speed": 25, "lateral_acceleration": 6, "acceleration": 4,
 "deceleration": 7, "lookahead_distance": 5, "lookahead_time": 0.5,
 "shift_speeds": [12, 18, 24, 30, 36]}
\endcode
*/
class Driver : public controller::Base {
public:
  Driver()
    : Base() {
    Initialize();
  }

  Driver(std::string name)
    : Base(name) {
    Initialize();
  }

  virtual ~Driver() override {}

  static std::unique_ptr<Driver> Create();

  static bool is_registered() {
    return registered_;
  }

  void DeclareParameters() override;
  void Connect() override;
  void Service(const double* y, double* dy) override;
  void UpdateOutputs(double t, const double* y) override;

  /*! \brief the length of the path (m) */
  double get_length() const {
    return stations_.empty() ? 0.0 : stations_.back();
  }

  /*! \brief the target speed of the profile at station s (m/s) */
  double ProfileSpeed(double s) const;

  /*! \brief the point of the path at station s; beyond the ends of an open
   * path, along its first or last segment */
  void PointAt(double s, double& x, double& y) const;

  /*! \brief the station of the point of the path nearest x, y, and the
   * distance of x, y to the right of the path (m) */
  double Nearest(double x, double y, double& lateral) const;

  /*! \brief the outputs for the pose and the speed along its x axis of the
   * vehicle's frame */
  void Drive(const ksl_SE3_t& pose, double speed);

  /*! \brief the signed curvature of the path at station s (1/m), positive
   * where it turns right (about z, down) */
  double CurvatureAt(double s) const;

  double get_steering() const {
    return steering_;
  }
  double get_throttle() const {
    return throttle_;
  }
  double get_brake() const {
    return brake_;
  }
  double get_gear() const {
    return gear_;
  }
  double get_target_speed() const {
    return target_speed_;
  }
  double get_station() const {
    return station_;
  }
  double get_lateral_error() const {
    return lateral_error_;
  }

protected:
  /*! \brief the outputs for the pose of the vehicle's frame, the velocity
   * of its origin in the fixed frame and its rate of yaw, about its z axis
   * (rad/s): those of Drive, from the speed along the frame's x axis */
  virtual void DriveMotion(const ksl_SE3_t& pose, const ksl_vec3_t& velocity,
                           double yaw_rate);
  /*! \brief the greatest speed of the profile on the signed curvature
   * (1/m), before the accelerations: \f$\sqrt{a_{lat} / |\kappa|}\f$ with
   * lateral_acceleration, unbounded on a straight */
  virtual double CornerSpeed(double curvature) const;

private:
  void Initialize() {
    type_ = "driver";
    DeclareParameters();
  }
  /*! \brief the pose and speed of the frame, and the outputs */
  void Compute();
  /*! \brief a frame of the body in the kinematic tree, whose pose is
   * computed with the motion (known once the model is set up, after
   * Connect), and the offset of the frame from it */
  void FindAnchor();
  /*! \brief the station s brought into the path: wrapped if closed */
  double Wrap(double s) const;

  static bool registered_;

protected:
  std::vector<std::string> body_frame_pair_;
  std::vector<double> path_;
  bool closed_ = false;
  double wheelbase_ = 0.0;
  double lookahead_distance_ = 5.0;
  double lookahead_time_ = 0.5;
  double max_steering_angle_ = 0.6;
  double speed_ = 10.0;
  double lateral_acceleration_ = 4.0;
  double acceleration_ = 3.0;
  double deceleration_ = 5.0;
  double throttle_gain_ = 0.5;
  double brake_gain_ = 0.3;
  std::vector<double> shift_speeds_;

  body::Base* body_ = nullptr;
  Frame* frame_ = nullptr;
  Frame* anchor_ = nullptr; //!< a frame of the body in the kinematic tree
  ksl_SE3_t offset_;        //!< frame_ in the coordinates of anchor_
  std::vector<double> x_, y_;      //!< the points, the first repeated if closed
  std::vector<double> stations_;   //!< station of each point
  std::vector<double> profile_;    //!< target speed at each point
  std::vector<double> curvature_;  //!< signed curvature at each point (1/m)

  double steering_ = 0.0;
  double throttle_ = 0.0;
  double brake_ = 0.0;
  double gear_ = 1.0;
  double target_speed_ = 0.0;
  double measured_speed_ = 0.0;
  double station_ = 0.0;
  double lateral_error_ = 0.0;
};

} // namespace controller
} // namespace core
} // namespace shift

#endif
