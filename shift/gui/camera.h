#ifndef SHIFT_GUI_CAMERA_H_
#define SHIFT_GUI_CAMERA_H_

#include <QMatrix4x4>

#include "ksl/SE3.h"
#include "ksl/matrix.h"
#include "ksl/vec3.h"

namespace shift {
namespace gui {

/*!
@brief Turntable camera orbiting a target point

Uses the shift world convention: +z points down (along gravity), so the
camera's up vector is -z. Azimuth is measured about the z axis from +x
towards +y, elevation is the angle above the x-y plane (towards -z).

A rigid motion can carry the whole turntable along, see set_follow. The
getters and the view are in world coordinates with the motion applied;
orbiting, panning and zooming then work in the moving frame.

Vectors and rigid transforms are KSL's; only the perspective projection is a
Qt matrix.
*/
class Camera {
public:
  Camera() {
    ksl_SE3f_setIdentity(&follow_);
  }

  ksl_vec3f_t get_position() const;
  ksl_vec3f_t get_forward() const;
  ksl_vec3f_t get_right() const;
  ksl_vec3f_t get_up() const;

  /*! \brief the view, from world coordinates to the camera's: x to the
   * right, y up and z backwards, as OpenGL's eye coordinates */
  ksl_SE3f_t get_view_matrix() const;
  QMatrix4x4 get_projection_matrix(float aspect) const;

  float get_distance() const {
    return distance_;
  }

  float get_fov_y() const {
    return fov_y_;
  }

  /*! \brief the rigid motion that carries the turntable, see set_follow */
  const ksl_SE3f_t& get_follow() const {
    return follow_;
  }

  /*! \brief carry the turntable by a rigid motion, from the coordinates of
   * the turntable to world coordinates. To hold the transform from the camera
   * to a body, this is the body's pose times the inverse of its pose when the
   * camera started following, so the camera keeps its place on the body,
   * rotation included. */
  void set_follow(const ksl_SE3f_t& motion) {
    follow_ = motion;
  }

  /*! \brief the frame on the ground under a body: at its x and y, at the
   * height ground_z, and turned by its heading, with no pitch or roll. A
   * camera that follows this frame sees the body bounce and tilt. */
  static ksl_SE3f_t GroundFrame(const ksl_SE3f_t& pose, float ground_z);

  /*! \brief stop following, leaving the camera where it is and looking the
   * same way. A turntable has no roll, so the view levels out. */
  void ClearFollow();

  /*! \brief look at the target from azimuth and elevation, in radians */
  void set_direction(float azimuth, float elevation);

  /*! \brief rotate about the target; arguments in radians */
  void Orbit(float delta_azimuth, float delta_elevation);

  /*! \brief translate the target in the view plane; arguments are fractions
   * of the viewport height */
  void Pan(float dx, float dy);

  /*! \brief scale the distance to the target by factor */
  void Zoom(float factor);
  /*! \brief move the camera towards (factor < 1) or away from (factor > 1)
   * a point in view, scaling its distance to the point by factor, without
   * turning: the point stays at the same place on the screen. The target
   * moves to the point's depth, so that the clipping planes follow the
   * distance to the point and later zooms and orbits are about it. */
  void ZoomTowards(const ksl_vec3f_t& point, float factor);

  /*! \brief place the target at the center of the box and set the distance
   * so the whole box is in view */
  void Frame(const ksl_vec3f_t& box_min, const ksl_vec3f_t& box_max);

private:
  /* the view direction and its axes, in the coordinates of the turntable */
  ksl_vec3f_t TurntableForward() const;
  ksl_vec3f_t TurntableRight() const;
  ksl_vec3f_t TurntableUp() const;
  /* the camera's position in the coordinates of the turntable */
  ksl_vec3f_t TurntablePosition() const;

  ksl_SE3f_t follow_; /*!< see set_follow; identity if not following */
  ksl_vec3f_t target_ = ksl_vec3f(0.f, 0.f, 0.f);
  float distance_ = 5.f;
  float azimuth_ = 0.6f;
  float elevation_ = 0.35f;
  float fov_y_ = 45.f; /*!< vertical field of view in degrees */
  float distance_min_ = 1e-3f;
  float distance_max_ = 1e4f;
  float radius_ = 1.f; /*!< radius of the framed box, see Frame */
};

/*! \brief a rigid transform as a 4 by 4 matrix, to combine with a
 * projection */
QMatrix4x4 ToQMatrix4x4(const ksl_SE3f_t& d);

} // namespace gui
} // namespace shift

#endif
