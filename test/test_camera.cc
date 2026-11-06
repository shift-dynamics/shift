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

/* a body's pose: a turn about each axis and a translation */
static QMatrix4x4 Pose(float x, float y, float z, float yaw, float pitch,
                       float roll) {
  QMatrix4x4 pose;
  pose.translate(x, y, z);
  pose.rotate(yaw, 0.f, 0.f, 1.f);
  pose.rotate(pitch, 0.f, 1.f, 0.f);
  pose.rotate(roll, 1.f, 0.f, 0.f);
  return pose;
}

static void ExpectNear(const QMatrix4x4& a, const QMatrix4x4& b, float limit) {
  for(int i = 0; i < 16; i++) {
    EXPECT_NEAR(a.constData()[i], b.constData()[i], limit) << "element " << i;
  }
}

static void ExpectNear(const QVector3D& a, const QVector3D& b, float limit) {
  EXPECT_LT((a - b).length(), limit);
}

TEST(Camera, FollowHoldsTheTransformFromTheCameraToTheBody) {
  shift::gui::Camera camera;
  camera.Frame(QVector3D(-5.f, -5.f, -5.f), QVector3D(5.f, 5.f, 5.f));
  const QMatrix4x4 start = Pose(1.f, 2.f, -1.f, 10.f, -5.f, 3.f);
  const QMatrix4x4 held = camera.get_view_matrix() * start;

  const QMatrix4x4 pose = Pose(120.f, -40.f, 2.f, 130.f, 12.f, -9.f);
  camera.set_follow(pose * start.inverted());
  ExpectNear(camera.get_view_matrix() * pose, held, 1e-3f);
}

TEST(Camera, FollowGettersAgreeWithTheViewMatrix) {
  shift::gui::Camera camera;
  camera.Frame(QVector3D(-5.f, -5.f, -5.f), QVector3D(5.f, 5.f, 5.f));
  camera.set_follow(Pose(30.f, 7.f, 1.f, 75.f, 20.f, -15.f));
  const QMatrix4x4 view = camera.get_view_matrix();
  ExpectNear(view.map(camera.get_position()), QVector3D(), 1e-3f);
  // the camera looks along -z of the view, with x right and y up
  ExpectNear(view.mapVector(camera.get_forward()), QVector3D(0.f, 0.f, -1.f),
             1e-4f);
  ExpectNear(view.mapVector(camera.get_right()), QVector3D(1.f, 0.f, 0.f),
             1e-4f);
  ExpectNear(view.mapVector(camera.get_up()), QVector3D(0.f, 1.f, 0.f), 1e-4f);
}

TEST(Camera, OrbitAndPanWorkInTheFrameFollowed) {
  shift::gui::Camera still, moving;
  for(auto* camera : {&still, &moving}) {
    camera->Frame(QVector3D(-5.f, -5.f, -5.f), QVector3D(5.f, 5.f, 5.f));
  }
  const QMatrix4x4 motion = Pose(50.f, -20.f, 3.f, 40.f, 15.f, 25.f);
  moving.set_follow(motion);
  for(auto* camera : {&still, &moving}) {
    camera->Orbit(0.4f, -0.2f);
    camera->Pan(0.1f, 0.05f);
    camera->Zoom(0.8f);
  }
  ExpectNear(moving.get_view_matrix() * motion, still.get_view_matrix(), 1e-3f);
}

TEST(Camera, ZoomTowardsAPointWhileFollowing) {
  shift::gui::Camera camera;
  camera.Frame(QVector3D(-5.f, -5.f, -5.f), QVector3D(5.f, 5.f, 5.f));
  camera.set_follow(Pose(50.f, -20.f, 3.f, 40.f, 15.f, 25.f));
  const QVector3D point = camera.get_position() + 8.f * camera.get_forward() +
                          1.f * camera.get_right();
  const QVector3D before = Screen(camera, point);
  camera.ZoomTowards(point, 0.5f);
  const QVector3D after = Screen(camera, point);
  EXPECT_NEAR(after.x(), before.x(), 1e-3f);
  EXPECT_NEAR(after.y(), before.y(), 1e-3f);
  EXPECT_NEAR(Depth(camera, point), 4.f, 1e-3f);
}

TEST(Camera, ClearFollowLeavesTheCameraWhereItIs) {
  shift::gui::Camera camera;
  camera.Frame(QVector3D(-5.f, -5.f, -5.f), QVector3D(5.f, 5.f, 5.f));
  camera.set_follow(Pose(50.f, -20.f, 3.f, 40.f, 15.f, 25.f));
  const QVector3D position = camera.get_position();
  const QVector3D forward = camera.get_forward();
  camera.ClearFollow();
  ExpectNear(camera.get_position(), position, 1e-3f);
  ExpectNear(camera.get_forward(), forward, 1e-4f);
  EXPECT_TRUE(camera.get_follow().isIdentity());
}

TEST(Camera, GroundFrameIgnoresBounceAndTilt) {
  const QMatrix4x4 level = Pose(40.f, -12.f, -0.9f, 65.f, 0.f, 0.f);
  // the same place and heading, higher up, pitched and rolled
  const QMatrix4x4 bounced = Pose(40.f, -12.f, -1.2f, 65.f, 6.f, -4.f);
  const QMatrix4x4 frame = shift::gui::Camera::GroundFrame(level, 0.3f);
  ExpectNear(shift::gui::Camera::GroundFrame(bounced, 0.3f), frame, 2e-3f);
  ExpectNear(frame.map(QVector3D()), QVector3D(40.f, -12.f, 0.3f), 1e-4f);
  ExpectNear(frame.mapVector(QVector3D(0.f, 0.f, 1.f)),
             QVector3D(0.f, 0.f, 1.f), 1e-6f);
}

TEST(Camera, GroundFrameFollowsTheHeading) {
  const QMatrix4x4 frame = shift::gui::Camera::GroundFrame(
    Pose(0.f, 0.f, 0.f, 90.f, 0.f, 0.f), 0.f);
  ExpectNear(frame.mapVector(QVector3D(1.f, 0.f, 0.f)),
             QVector3D(0.f, 1.f, 0.f), 1e-5f);
}
