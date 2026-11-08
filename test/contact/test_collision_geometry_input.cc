/* collision geometry from the model input: given by its dimensions and placed
by a pose, or fitted to the body's meshes (all active ones, or those named in
fit_to) */
#include "gtest/gtest.h"

#include <cstdio>
#include <fstream>
#include <string>

#include "shift/config.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/collision_geometry/ellipsoid.h"
#include "shift/core/collision_geometry/sphere.h"
#include "shift/core/context.h"

/* a model of free bodies, given as JSON text for the bodies and joints */
static std::string WriteModel(const std::string& name,
                              const std::string& bodies,
                              const std::string& joints) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  std::ofstream file(path);
  file << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "include_directories": [")"
       << SHIFT_EXAMPLES_DIR << R"(/collision", ")" << SHIFT_EXAMPLES_DIR
       << R"(/brake"],
    "enable_contact_detection": true,
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "rk4", "time_span": 0.01, "time_step": 0.01}]
  },
  "assemblies": [{"name": "input", "bodies": [)"
       << bodies << R"(], "joints": [)" << joints << R"(]}]
})";
  return path;
}

/* load the model, then remove its file, also when loading throws */
static void Load(shift::core::Context& context, const std::string& path) {
  try {
    context.LoadJSON(path);
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
  std::remove(path.c_str());
}

static std::string Body(const std::string& name, const std::string& geometry,
                        const std::string& collision_geometry) {
  return R"({"name": ")" + name + R"(", "geometry": [)" + geometry +
         R"(], "collision_geometry": [)" + collision_geometry +
         R"(], "center_of_mass": [0, 0, 0], "inertia": {"mass": 1.0,
         "Ixx": 1.0, "Iyy": 1.0, "Izz": 1.0, "Ixy": 0.0, "Iyz": 0.0,
         "Izx": 0.0}})";
}

static std::string FloatJoint(const std::string& body) {
  return R"({"name": ")" + body + R"(_joint", "type": "float",
    "parent": [["fixed"], "origin"], "child": [[")" +
         body + R"("], "origin"],
    "initial_velocity": [0, 0, 0, 0, 0, 0]})";
}

static const std::string kBunny =
  R"({"name": "visual", "type": "mesh", "filename": ["stanford_bunny.obj"]})";

static const shift::core::collision_geometry::Base* Find(const shift::core::Context& context,
                        const std::string& name) {
  for(auto geometry : context.get_contact_geometry()) {
    if(geometry->get_name() == name) {
      return geometry;
    }
  }
  return nullptr;
}

static double Extent(const shift::core::collision_geometry::Base& g, const ksl_vec3_t& d) {
  ksl_vec3_t local, global;
  int hint = 0;
  g.Support(d, local, global, hint);
  return global.x * d.x + global.y * d.y + global.z * d.z;
}

TEST(CollisionGeometryInput, DimensionsAndPose) {
  const std::string collision = R"(
    {"name": "ball", "type": "sphere", "radius": 0.2,
     "pose": {"T": [0.1, 0.2, 0.3]}},
    {"name": "egg", "type": "ellipsoid", "semi_axes": [0.3, 0.2, 0.1],
     "pose": {"T": [0.0, 0.0, -0.5],
              "R": [[0, -1, 0], [1, 0, 0], [0, 0, 1]]}},
    {"name": "box", "type": "convex_hull", "half_extents": [0.1, 0.2, 0.3],
     "pose": {"T": [1.0, 0.0, 0.0]}},
    {"name": "origin_ball", "type": "sphere", "radius": 0.05})";
  shift::core::Context context;
  Load(context, WriteModel("dimensions", Body("probe", kBunny, collision),
                              FloatJoint("probe")));
  const ksl_SE3_t identity = {{{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}}};

  auto ball = Find(context, "ball");
  ASSERT_NE(ball, nullptr);
  EXPECT_DOUBLE_EQ(
    static_cast<const shift::core::collision_geometry::Sphere*>(ball)
      ->get_radius(),
    0.2);
  EXPECT_DOUBLE_EQ(ball->get_d_wrt_body().t.x, 0.1);
  EXPECT_DOUBLE_EQ(ball->get_d_wrt_body().t.y, 0.2);
  EXPECT_DOUBLE_EQ(ball->get_d_wrt_body().t.z, 0.3);
  EXPECT_DOUBLE_EQ(ball->get_bounding_radius(), 0.2);

  // the sphere is centered at the body origin without a pose, whatever the
  // mesh
  auto origin_ball = Find(context, "origin_ball");
  ASSERT_NE(origin_ball, nullptr);
  for(int k = 0; k < 3; k++) {
    EXPECT_EQ(origin_ball->get_d_wrt_body().t.at[k], 0.0);
  }

  // the ellipsoid semi axes are along the axes of the pose: its x axis is
  // the body y axis
  auto egg = Find(context, "egg");
  ASSERT_NE(egg, nullptr);
  auto ellipsoid =
    static_cast<const shift::core::collision_geometry::Ellipsoid*>(egg);
  EXPECT_DOUBLE_EQ(ellipsoid->get_semi_axes().x, 0.3);
  EXPECT_DOUBLE_EQ(egg->get_d_wrt_body().t.z, -0.5);
  const_cast<shift::core::collision_geometry::Base*>(egg)->UpdatePose(identity);
  EXPECT_NEAR(Extent(*egg, {{0, 1, 0}}), 0.3, 1e-12);
  EXPECT_NEAR(Extent(*egg, {{1, 0, 0}}), 0.2, 1e-12);
  EXPECT_NEAR(Extent(*egg, {{0, 0, 1}}), -0.5 + 0.1, 1e-12);

  auto box = Find(context, "box");
  ASSERT_NE(box, nullptr);
  const_cast<shift::core::collision_geometry::Base*>(box)->UpdatePose(identity);
  EXPECT_NEAR(Extent(*box, {{1, 0, 0}}), 1.1, 1e-12);
  EXPECT_NEAR(Extent(*box, {{-1, 0, 0}}), -0.9, 1e-12);
  EXPECT_NEAR(Extent(*box, {{0, 1, 0}}), 0.2, 1e-12);
  EXPECT_NEAR(Extent(*box, {{0, 0, -1}}), 0.3, 1e-12);
}

TEST(CollisionGeometryInput, FitToNamedMeshes) {
  // the probe draws the bunny but collides with the hull of the bridge, an
  // inactive (not drawn) mesh; the reference body draws and fits the bridge
  const std::string bridge =
    R"({"name": "collision_mesh", "type": "mesh", "active": false,
        "filename": ["bridge.obj"]})";
  const std::string bodies =
    Body("probe", kBunny + "," + bridge,
         R"({"name": "probe_hull", "type": "convex_hull",
             "fit_to": ["collision_mesh"]})") +
    "," +
    Body("reference",
         R"({"name": "visual", "type": "mesh", "filename": ["bridge.obj"]})",
         R"({"name": "reference_hull", "type": "convex_hull"})") +
    "," +
    Body("bunny", kBunny, R"({"name": "bunny_hull", "type": "convex_hull"})");
  shift::core::Context context;
  Load(context, WriteModel("fit_to", bodies,
                              FloatJoint("probe") + "," +
                                FloatJoint("reference") + "," +
                                FloatJoint("bunny")));
  auto probe = Find(context, "probe_hull");
  auto reference = Find(context, "reference_hull");
  auto bunny = Find(context, "bunny_hull");
  ASSERT_NE(probe, nullptr);
  ASSERT_NE(reference, nullptr);
  ASSERT_NE(bunny, nullptr);
  EXPECT_DOUBLE_EQ(probe->get_bounding_radius(),
                   reference->get_bounding_radius());
  EXPECT_NE(probe->get_bounding_radius(), bunny->get_bounding_radius());
}

TEST(CollisionGeometryInput, InvalidCombinations) {
  struct Case {
    const char* name;
    std::string collision;
  };
  const Case cases[] = {
    {"dimensions_and_fit_to",
     R"({"name": "c", "type": "sphere", "radius": 0.1, "fit_to": ["visual"]})"},
    {"fitted_with_pose",
     R"({"name": "c", "type": "convex_hull", "pose": {"T": [0, 0, 1]}})"},
    {"unknown_mesh",
     R"({"name": "c", "type": "convex_hull", "fit_to": ["missing"]})"},
    {"negative_radius", R"({"name": "c", "type": "sphere", "radius": -0.1})"},
    {"partial_semi_axes",
     R"({"name": "c", "type": "ellipsoid", "semi_axes": [0.1, 0.0, 0.1]})"},
  };
  for(const auto& c : cases) {
    shift::core::Context context;
    EXPECT_ANY_THROW(Load(context, WriteModel(
      c.name, Body("probe", kBunny, c.collision), FloatJoint("probe"))))
      << c.name;
  }
}
