#include "gtest/gtest.h"

#include <QMatrix4x4>
#include <QVector3D>

#include "shift/gui/camera.h"

/* where a world point appears on the screen, in normalized device
coordinates, and its depth along the view direction */
static QVector3D Screen(const shift::gui::Camera& camera,
                        const QVector3D& point) {
  return (camera.get_projection_matrix(1.5f) * camera.get_view_matrix())
    .map(point);
}

static float Depth(const shift::gui::Camera& camera, const QVector3D& point) {
  return QVector3D::dotProduct(point - camera.get_position(),
                               camera.get_forward());
}

/* zooming towards a point off the center of the view keeps it at the same
place on the screen, and the distance to the target becomes its depth, so the
near plane (1% of it) never reaches the point */
TEST(Camera, ZoomTowardsKeepsThePointInView) {
  shift::gui::Camera camera;
  camera.Frame(QVector3D(-5.f, -5.f, -5.f), QVector3D(5.f, 5.f, 5.f));
  // a point in view, off the center
  const QVector3D point = camera.get_position() +
                          camera.get_forward() * 4.f +
                          camera.get_right() * 0.5f + camera.get_up() * 0.3f;
  const QVector3D before = Screen(camera, point);
  for(int step = 0; step < 60; step++) {
    camera.ZoomTowards(point, 0.88f);
    const QVector3D after = Screen(camera, point);
    // to within the rounding of the camera position in single precision,
    // about 1e-6 m some 10 m from the origin, which shows when the camera is
    // a few millimeters from the point
    EXPECT_NEAR(after.x(), before.x(), 3e-3f) << step;
    EXPECT_NEAR(after.y(), before.y(), 3e-3f) << step;
    // inside the depth range: not clipped by the near or far plane
    EXPECT_GT(after.z(), -1.f) << step;
    EXPECT_LT(after.z(), 1.f) << step;
    EXPECT_NEAR(camera.get_distance(), Depth(camera, point),
                1e-3f * camera.get_distance())
      << step;
  }
  // 60 steps of 0.88 from a depth of 4: about 2 mm, still in view
  EXPECT_LT(camera.get_distance(), 0.01f);
}

/* up close, the far plane still reaches past the framed box */
TEST(Camera, FarPlaneCoversTheFramedBox) {
  shift::gui::Camera camera;
  camera.Frame(QVector3D(-5.f, -5.f, -5.f), QVector3D(5.f, 5.f, 5.f));
  const QVector3D point = camera.get_position() + camera.get_forward() * 4.f;
  for(int step = 0; step < 60; step++) {
    camera.ZoomTowards(point, 0.88f);
  }
  // the far corner of the box, seen past the point
  const QVector3D corner =
    camera.get_position() + camera.get_forward() * (Depth(camera, point) + 17.f);
  EXPECT_LT(Screen(camera, corner).z(), 1.f);
}
