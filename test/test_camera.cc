#include "gtest/gtest.h"

#include <QMatrix4x4>
#include <QVector3D>

#include <cmath>

#include "ksl/mat3x3.h"
#include "ksl/matrix.h"
#include "ksl/SE3.h"
#include "ksl/vec3.h"
#include "shift/gui/camera.h"

/* the point a + k b */
static ksl_vec3f_t Along(const ksl_vec3f_t& a, float k, const ksl_vec3f_t& b) {
  ksl_vec3f_t p = a;
  ksl_axpy_vvf(k, &b, &p);
  return p;
}

/* where a world point appears on the screen, in normalized device
coordinates, and its depth along the view direction (the projection is
Qt's) */
static QVector3D Screen(const shift::gui::Camera& camera,
                        const ksl_vec3f_t& point) {
  return (camera.get_projection_matrix(1.5f) *
          shift::gui::ToQMatrix4x4(camera.get_view_matrix()))
    .map(QVector3D(point.x, point.y, point.z));
}

static float Depth(const shift::gui::Camera& camera, const ksl_vec3f_t& point) {
  const ksl_vec3f_t position = camera.get_position();
  const ksl_vec3f_t forward = camera.get_forward();
  ksl_vec3f_t d;
  ksl_subtract_vvf(&point, &position, &d);
  return ksl_dot_vvf(&d, &forward);
}

static void FrameTheBox(shift::gui::Camera& camera) {
  camera.Frame(ksl_vec3f(-5.f, -5.f, -5.f), ksl_vec3f(5.f, 5.f, 5.f));
}

/* zooming towards a point off the center of the view keeps it at the same
place on the screen, and the distance to the target becomes its depth, so the
near plane (1% of it) never reaches the point */
TEST(Camera, ZoomTowardsKeepsThePointInView) {
  shift::gui::Camera camera;
  FrameTheBox(camera);
  // a point in view, off the center
  const ksl_vec3f_t point =
    Along(Along(Along(camera.get_position(), 4.f, camera.get_forward()), 0.5f,
                camera.get_right()),
          0.3f, camera.get_up());
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
  FrameTheBox(camera);
  const ksl_vec3f_t point =
    Along(camera.get_position(), 4.f, camera.get_forward());
  for(int step = 0; step < 60; step++) {
    camera.ZoomTowards(point, 0.88f);
  }
  // the far corner of the box, seen past the point
  const ksl_vec3f_t corner = Along(
    camera.get_position(), Depth(camera, point) + 17.f, camera.get_forward());
  EXPECT_LT(Screen(camera, corner).z(), 1.f);
}

/* a body's pose: a translation, then turns about z, y and x (degrees) */
static ksl_SE3f_t Pose(float x, float y, float z, float yaw, float pitch,
                       float roll) {
  const float degree = float(M_PI) / 180.f;
  const ksl_vec3f_t ex = ksl_vec3f(1.f, 0.f, 0.f);
  const ksl_vec3f_t ey = ksl_vec3f(0.f, 1.f, 0.f);
  const ksl_vec3f_t ez = ksl_vec3f(0.f, 0.f, 1.f);
  ksl_mat3x3f_t rz, ry, rx, rzy;
  ksl_mat3x3f_setFromAxisAngle(&rz, &ez, yaw * degree);
  ksl_mat3x3f_setFromAxisAngle(&ry, &ey, pitch * degree);
  ksl_mat3x3f_setFromAxisAngle(&rx, &ex, roll * degree);
  ksl_product_drdrf(&rz, &ry, &rzy);
  ksl_SE3f_t pose;
  ksl_product_drdrf(&rzy, &rx, &pose.R);
  pose.t = ksl_vec3f(x, y, z);
  return pose;
}

/* a b */
static ksl_SE3f_t Product(const ksl_SE3f_t& a, const ksl_SE3f_t& b) {
  ksl_SE3f_t ab;
  ksl_product_ddf(&a, &b, &ab);
  return ab;
}

static void ExpectNear(const ksl_SE3f_t& a, const ksl_SE3f_t& b, float limit) {
  for(int i = 0; i < 12; i++) {
    EXPECT_NEAR(a.as_array[i], b.as_array[i], limit) << "element " << i;
  }
}

static void ExpectNear(const ksl_vec3f_t& a, const ksl_vec3f_t& b,
                       float limit) {
  ksl_vec3f_t d;
  ksl_subtract_vvf(&a, &b, &d);
  EXPECT_LT(ksl_vec3f_l2norm(&d), limit);
}

/* the point or the direction transformed by d */
static ksl_vec3f_t Point(const ksl_SE3f_t& d, const ksl_vec3f_t& p) {
  ksl_vec3f_t q;
  ksl_product_dvf(&d, &p, &q);
  return q;
}

static ksl_vec3f_t Direction(const ksl_SE3f_t& d, const ksl_vec3f_t& v) {
  ksl_vec3f_t w;
  ksl_product_drvf(&d.R, &v, &w);
  return w;
}

TEST(Camera, FollowHoldsTheTransformFromTheCameraToTheBody) {
  shift::gui::Camera camera;
  FrameTheBox(camera);
  const ksl_SE3f_t start = Pose(1.f, 2.f, -1.f, 10.f, -5.f, 3.f);
  const ksl_SE3f_t held = Product(camera.get_view_matrix(), start);

  const ksl_SE3f_t pose = Pose(120.f, -40.f, 2.f, 130.f, 12.f, -9.f);
  ksl_SE3f_t motion;
  ksl_product_ddinvf(&pose, &start, &motion);
  camera.set_follow(motion);
  ExpectNear(Product(camera.get_view_matrix(), pose), held, 1e-3f);
}

TEST(Camera, FollowGettersAgreeWithTheViewMatrix) {
  shift::gui::Camera camera;
  FrameTheBox(camera);
  camera.set_follow(Pose(30.f, 7.f, 1.f, 75.f, 20.f, -15.f));
  const ksl_SE3f_t view = camera.get_view_matrix();
  ExpectNear(Point(view, camera.get_position()), ksl_vec3f(0.f, 0.f, 0.f),
             1e-3f);
  // the camera looks along -z of the view, with x right and y up
  ExpectNear(Direction(view, camera.get_forward()), ksl_vec3f(0.f, 0.f, -1.f),
             1e-4f);
  ExpectNear(Direction(view, camera.get_right()), ksl_vec3f(1.f, 0.f, 0.f),
             1e-4f);
  ExpectNear(Direction(view, camera.get_up()), ksl_vec3f(0.f, 1.f, 0.f),
             1e-4f);
}

/* the view agrees with Qt's lookAt from the same position, target and up */
TEST(Camera, ViewIsLookAt) {
  shift::gui::Camera camera;
  FrameTheBox(camera);
  camera.Orbit(0.7f, 0.3f);
  const ksl_vec3f_t p = camera.get_position();
  const ksl_vec3f_t target = Along(p, camera.get_distance(),
                                   camera.get_forward());
  QMatrix4x4 look;
  look.lookAt(QVector3D(p.x, p.y, p.z),
              QVector3D(target.x, target.y, target.z),
              QVector3D(0.f, 0.f, -1.f));
  const QMatrix4x4 view = shift::gui::ToQMatrix4x4(camera.get_view_matrix());
  for(int i = 0; i < 16; i++) {
    EXPECT_NEAR(view.constData()[i], look.constData()[i], 1e-5f) << i;
  }
}

TEST(Camera, OrbitAndPanWorkInTheFrameFollowed) {
  shift::gui::Camera still, moving;
  for(auto* camera : {&still, &moving}) {
    FrameTheBox(*camera);
  }
  const ksl_SE3f_t motion = Pose(50.f, -20.f, 3.f, 40.f, 15.f, 25.f);
  moving.set_follow(motion);
  for(auto* camera : {&still, &moving}) {
    camera->Orbit(0.4f, -0.2f);
    camera->Pan(0.1f, 0.05f);
    camera->Zoom(0.8f);
  }
  ExpectNear(Product(moving.get_view_matrix(), motion),
             still.get_view_matrix(), 1e-3f);
}

TEST(Camera, ZoomTowardsAPointWhileFollowing) {
  shift::gui::Camera camera;
  FrameTheBox(camera);
  camera.set_follow(Pose(50.f, -20.f, 3.f, 40.f, 15.f, 25.f));
  const ksl_vec3f_t point =
    Along(Along(camera.get_position(), 8.f, camera.get_forward()), 1.f,
          camera.get_right());
  const QVector3D before = Screen(camera, point);
  camera.ZoomTowards(point, 0.5f);
  const QVector3D after = Screen(camera, point);
  EXPECT_NEAR(after.x(), before.x(), 1e-3f);
  EXPECT_NEAR(after.y(), before.y(), 1e-3f);
  EXPECT_NEAR(Depth(camera, point), 4.f, 1e-3f);
}

TEST(Camera, ClearFollowLeavesTheCameraWhereItIs) {
  shift::gui::Camera camera;
  FrameTheBox(camera);
  camera.set_follow(Pose(50.f, -20.f, 3.f, 40.f, 15.f, 25.f));
  const ksl_vec3f_t position = camera.get_position();
  const ksl_vec3f_t forward = camera.get_forward();
  camera.ClearFollow();
  ExpectNear(camera.get_position(), position, 1e-3f);
  ExpectNear(camera.get_forward(), forward, 1e-4f);
  ksl_SE3f_t identity;
  ksl_SE3f_setIdentity(&identity);
  ExpectNear(camera.get_follow(), identity, 0.f);
}

TEST(Camera, GroundFrameIgnoresBounceAndTilt) {
  const ksl_SE3f_t level = Pose(40.f, -12.f, -0.9f, 65.f, 0.f, 0.f);
  // the same place and heading, higher up, pitched and rolled
  const ksl_SE3f_t bounced = Pose(40.f, -12.f, -1.2f, 65.f, 6.f, -4.f);
  const ksl_SE3f_t frame = shift::gui::Camera::GroundFrame(level, 0.3f);
  ExpectNear(shift::gui::Camera::GroundFrame(bounced, 0.3f), frame, 2e-3f);
  ExpectNear(Point(frame, ksl_vec3f(0.f, 0.f, 0.f)),
             ksl_vec3f(40.f, -12.f, 0.3f), 1e-4f);
  ExpectNear(Direction(frame, ksl_vec3f(0.f, 0.f, 1.f)),
             ksl_vec3f(0.f, 0.f, 1.f), 1e-6f);
}

TEST(Camera, GroundFrameFollowsTheHeading) {
  const ksl_SE3f_t frame = shift::gui::Camera::GroundFrame(
    Pose(0.f, 0.f, 0.f, 90.f, 0.f, 0.f), 0.f);
  ExpectNear(Direction(frame, ksl_vec3f(1.f, 0.f, 0.f)),
             ksl_vec3f(0.f, 1.f, 0.f), 1e-5f);
}
