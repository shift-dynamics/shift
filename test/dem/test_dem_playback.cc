/* the grains of a DEM (force_module::Dem) in the viewer (gui::Simulation):
each shape of grain is a mesh and each grain an instance of it, its pose
after the bodies'; the grains recorded at each knot point are played back
forward and back; Reset starts them over as the model placed them, and the
run after it is the first again, and Run from a time shown starts them from
the grains shown. */

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/context.h"
#include "shift/core/force_module/dem.h"
#include "shift/gui/simulation.h"

/* a plate of 0.1 kg on a vertical slide drops onto beads and two-sphere
clumps on a floor, in a box of walls, on the CPU */
static const char* kModel = R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": 0.05, "time_step": 0.01,
                     "hmax": 4e-5}]
  },
  "assemblies": [{"name": "a",
    "force_modules": [{"type": "dem", "name": "grains", "device": "cpu",
      "materials": [{"name": "glass", "youngs_modulus": 5e7,
                     "poisson_ratio": 0.3, "restitution": 0.3,
                     "friction": 0.5, "rolling_resistance": 0.05}],
      "grains": [{"name": "bead", "radius": 0.005, "material": "glass",
                  "mass": 0.0013},
                 {"name": "pair", "mass": 0.0018,
                  "inertia": [1.6e-8, 3.4e-8, 3.4e-8],
                  "spheres": [{"center": [-0.003, 0, 0], "radius": 0.004,
                               "material": "glass"},
                              {"center": [0.003, 0, 0], "radius": 0.004,
                               "material": "glass"}]}],
      "fill": [{"grain": "bead", "min": [-0.022, -0.022, -0.011],
                "max": [0.022, 0.022, 0], "spacing": 0.011, "jitter": 0.05,
                "seed": 7},
               {"grain": "pair", "min": [-0.022, -0.022, -0.022],
                "max": [0.022, 0.022, -0.011], "spacing": 0.011,
                "jitter": 0.05, "seed": 8}],
      "walls": [{"normal": [0, 0, -1], "offset": 0, "material": "glass"},
                {"normal": [1, 0, 0], "offset": -0.03, "material": "glass"},
                {"normal": [-1, 0, 0], "offset": -0.03, "material": "glass"},
                {"normal": [0, 1, 0], "offset": -0.03, "material": "glass"},
                {"normal": [0, -1, 0], "offset": -0.03, "material": "glass"}],
      "bodies": [{"body": "plate", "geometry": ["plate"],
                  "material": "glass"}]}],
    "bodies": [{"name": "plate", "center_of_mass": [0, 0, -0.035],
                "collision_geometry": [{"name": "plate", "type": "box",
                  "half_extents": [0.025, 0.025, 0.005], "contact": false,
                  "pose": {"T": [0, 0, -0.035]}}],
                "inertia": {"mass": 0.1, "Ixx": 1e-4, "Iyy": 1e-4,
                            "Izz": 1e-4, "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [{"name": "slide", "type": "prismatic",
                "parent": [["fixed"], "origin"],
                "child": [["plate"], "origin"], "axis": [0, 0, 1]}]
  }]
})";

class DemPlayback : public testing::Test {
protected:
  void SetUp() override {
    path_ = testing::TempDir() + "/dem_playback.json";
    {
      std::ofstream file(path_);
      file << kModel;
    }
    simulation_.Load(path_);
    for(auto module : simulation_.get_context()->get_active_force_modules()) {
      if(auto dem = dynamic_cast<shift::core::force_module::Dem*>(module)) {
        dem_ = dem;
      }
    }
    ASSERT_NE(dem_, nullptr);
  }
  void TearDown() override {
    std::remove(path_.c_str());
  }

  void RunToEnd() {
    simulation_.Run();
    while(simulation_.is_computing()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_EQ(simulation_.get_error(), "");
  }

  /* the centers of the grains drawn */
  std::vector<float> Shown() {
    shift::gui::Poses poses;
    simulation_.TakePoses(poses);
    const auto& scene = simulation_.get_scene();
    EXPECT_EQ(poses.bodies.size(),
              size_t(scene.body_count) + size_t(scene.grain_count));
    std::vector<float> centers;
    for(size_t k = size_t(scene.body_count); k < poses.bodies.size(); k++) {
      for(int a = 0; a < 3; a++) {
        centers.push_back(poses.bodies[k](a, 3));
      }
    }
    return centers;
  }

  /* the centers of grains */
  static std::vector<float> Centers(const shift::core::dem::State& grains) {
    std::vector<float> centers;
    for(size_t g = 0; g < grains.size(); g++) {
      for(int a = 0; a < 3; a++) {
        centers.push_back(float(grains.position[g][a]));
      }
    }
    return centers;
  }

  std::vector<float> Grains() {
    shift::core::dem::State grains;
    dem_->GetGrains(grains);
    return Centers(grains);
  }

  std::string path_;
  shift::gui::Simulation simulation_;
  shift::core::force_module::Dem* dem_ = nullptr;
};

static float MaxDifference(const std::vector<float>& a,
                           const std::vector<float>& b) {
  float d = 0.0f;
  for(size_t k = 0; k < a.size(); k++) {
    d = std::max(d, std::fabs(a[k] - b[k]));
  }
  return d;
}

/* a mesh for each shape, its spheres' bounds, and an instance for each
grain, with its pose */
TEST_F(DemPlayback, Scene) {
  const auto& scene = simulation_.get_scene();
  const auto& initial = dem_->get_initial_grains();
  ASSERT_EQ(size_t(scene.grain_count), initial.size());
  ASSERT_EQ(initial.size(), 32u);
  std::vector<int> instances(scene.meshes.size(), 0);
  for(const auto& instance : scene.instances) {
    if(instance.body_index >= scene.body_count) {
      const size_t g = size_t(instance.body_index - scene.body_count);
      ASSERT_LT(g, initial.size());
      const auto& mesh = scene.meshes[size_t(instance.mesh_index)];
      /* the bead's bounds are its radius; the pair's half its length */
      const float half = initial.template_index[g] == 0 ? 0.005f : 0.007f;
      EXPECT_NEAR(mesh.bounds_max.x(), half, 1e-6f);
      EXPECT_NEAR(mesh.bounds_min.x(), -half, 1e-6f);
      instances[size_t(instance.mesh_index)]++;
    }
  }
  int shapes = 0;
  for(int count : instances) {
    if(count > 0) {
      EXPECT_EQ(count, 16);
      shapes++;
    }
  }
  EXPECT_EQ(shapes, 2);
  EXPECT_EQ(Shown(), Centers(initial));
}

/* the grains at the end of the run and at the start, forward and back */
TEST_F(DemPlayback, ForwardAndBack) {
  const std::vector<float> start = Centers(dem_->get_initial_grains());
  RunToEnd();
  const std::vector<float> end = Grains();
  ASSERT_GT(MaxDifference(end, start), 1e-3f);
  /* the last knot point, at or just past the time span */
  const double last = simulation_.get_computed_time();
  simulation_.ShowTime(last);
  EXPECT_EQ(Shown(), end);
  simulation_.ShowTime(0.0);
  EXPECT_EQ(Shown(), start);
  /* part way: neither, and the same when shown again */
  simulation_.ShowTime(0.015);
  const std::vector<float> early = Shown();
  EXPECT_GT(MaxDifference(early, start), 1e-4f);
  EXPECT_GT(MaxDifference(early, end), 1e-4f);
  simulation_.ShowTime(0.04);
  simulation_.ShowTime(0.015);
  EXPECT_EQ(Shown(), early);
  simulation_.ShowTime(last);
  EXPECT_EQ(Shown(), end);
}

/* Reset starts the grains over as they were placed, and the run after it is
the first one again; the grains' impulse on the plate starts from zero */
TEST_F(DemPlayback, ResetRepeatsTheRun) {
  RunToEnd();
  const std::vector<float> end = Grains();
  const double impulse = dem_->get_body_impulse(0).lin.z;
  ASSERT_LT(impulse, 0.0);
  simulation_.Reset();
  EXPECT_EQ(Shown(), Centers(dem_->get_initial_grains()));
  EXPECT_EQ(Grains(), Centers(dem_->get_initial_grains()));
  EXPECT_EQ(dem_->get_body_impulse(0).lin.z, 0.0);
  RunToEnd();
  EXPECT_EQ(Grains(), end);
  EXPECT_EQ(dem_->get_body_impulse(0).lin.z, impulse);
}

/* Run from a time shown starts the grains from those shown */
TEST_F(DemPlayback, RunFromShownGrains) {
  RunToEnd();
  simulation_.ShowTime(0.025);
  const std::vector<float> shown = Shown();
  ASSERT_TRUE(simulation_.AdoptShownState());
  EXPECT_EQ(Grains(), shown);
  RunToEnd();
  simulation_.ShowTime(0.0);
  EXPECT_EQ(Shown(), shown);
  simulation_.ShowTime(simulation_.get_computed_time());
  EXPECT_EQ(Shown(), Grains());
}
