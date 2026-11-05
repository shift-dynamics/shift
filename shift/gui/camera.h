#ifndef SHIFT_GUI_CAMERA_H_
#define SHIFT_GUI_CAMERA_H_

#include <QMatrix4x4>
#include <QVector3D>

namespace shift {
namespace gui {

/*!
@brief Turntable camera orbiting a target point

Uses the shift world convention: +z points down (along gravity), so the
camera's up vector is -z. Azimuth is measured about the z axis from +x
towards +y, elevation is the angle above the x-y plane (towards -z).
*/
class Camera {
public:
  QVector3D get_position() const;
  QVector3D get_forward() const;
  QVector3D get_right() const;
  QVector3D get_up() const;

  QMatrix4x4 get_view_matrix() const;
  QMatrix4x4 get_projection_matrix(float aspect) const;

  float get_distance() const {
    return distance_;
  }

  float get_fov_y() const {
    return fov_y_;
  }

  /*! \brief rotate about the target; arguments in radians */
  void Orbit(float delta_azimuth, float delta_elevation);

  /*! \brief translate the target in the view plane; arguments are fractions
   * of the viewport height */
  void Pan(float dx, float dy);

  /*! \brief scale the distance to the target by factor */
  void Zoom(float factor);

  /*! \brief place the target at the center of the box and set the distance
   * so the whole box is in view */
  void Frame(const QVector3D& box_min, const QVector3D& box_max);

private:
  QVector3D target_{0.f, 0.f, 0.f};
  float distance_ = 5.f;
  float azimuth_ = 0.6f;
  float elevation_ = 0.35f;
  float fov_y_ = 45.f; /*!< vertical field of view in degrees */
  float distance_min_ = 1e-3f;
  float distance_max_ = 1e4f;
};

} // namespace gui
} // namespace shift

#endif
