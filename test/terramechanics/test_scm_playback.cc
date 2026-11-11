/* a deformable terrain (terrain::Scm) in the viewer (gui::Simulation): the
levels recorded at each knot point as the nodes that changed, played back
forward and back, give the surface the terrain had at the end and at the
start; Reset returns to the undisturbed surface, and Run continues from the
levels shown. An HDF5 log (logger::HDF5) holds its model and the files it
read, and the viewer plays it back on its own. */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <filesystem>
#include <json/json.h>

#include <gtest/gtest.h>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/terrain/scm.h"
#include "shift/gui/simulation.h"

/* a box of 2000 kg on a vertical prismatic joint, its bottom on the surface
of an SCM grid of spacing 0.1 */
static const char* kModel = R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": 1.0, "time_step": 0.01,
                     "relative_error": 1e-8, "absolute_error": 1e-9}]
  },
  "frames": [{"name": "terrain", "T": [0, 0, 0]}],
  "terrain": [{"type": "scm", "name": "site", "parent": [["fixed"], "terrain"],
      "x": [-1, 1], "y": [-0.5, 0.5], "spacing": 0.1, "damping": 5,
      "soil": {"sinkage_exponent": 1.1, "cohesive_modulus": 2e4,
               "frictional_modulus": 1e6, "cohesion": 1000,
               "friction_angle": 0.5, "shear_modulus": 0.01}}],
  "assemblies": [{"name": "a",
    "force_modules": [{"type": "soil_contact", "name": "soil",
                       "bodies": ["plate"]}],
    "bodies": [{"name": "plate", "collision_geometry": [
                  {"name": "plate", "type": "box",
                   "half_extents": [0.25, 0.25, 0.05]}],
                "center_of_mass": [0, 0, 0],
                "inertia": {"mass": 2000, "Ixx": 40, "Iyy": 40, "Izz": 80,
                            "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [{"name": "slide", "type": "prismatic",
                "parent": [["fixed"], "origin"],
                "child": [["plate"], "origin"], "axis": [0, 0, 1],
                "initial_displacement": -0.05}]
  }]
})";

class ScmPlayback : public testing::Test {
protected:
  void SetUp() override {
    path_ = testing::TempDir() + "/scm_playback.json";
    {
      std::ofstream file(path_);
      file << kModel;
    }
    simulation_.Load(path_);
    scm_ = dynamic_cast<shift::core::terrain::Scm*>(
      simulation_.get_context()->get_active_terrain()[0]);
    ASSERT_NE(scm_, nullptr);
    ASSERT_EQ(simulation_.get_scene().terrains.size(), 1u);
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

  /* the levels drawn: control points less the undisturbed surface */
  std::vector<float> Shown() const {
    const auto& t = simulation_.get_scene().terrains[0];
    const long nx = scm_->get_node_count_x(), ny = scm_->get_node_count_y();
    EXPECT_EQ(t.width, nx + 2);
    EXPECT_EQ(t.height, ny + 2);
    std::vector<float> levels;
    for(long j = 0; j < ny; j++) {
      for(long i = 0; i < nx; i++) {
        levels.push_back(t.z[size_t((j + 1) * t.width + i + 1)] -
                         float(scm_->UndisturbedZ(i, j)));
      }
    }
    return levels;
  }

  /* the rectangle of TakeTerrainUpdates written over a copy of the control
  points uploaded before */
  void Upload(std::vector<float>& uploaded) {
    std::vector<shift::gui::TerrainUpdate> updates;
    simulation_.TakeTerrainUpdates(updates);
    const int width = simulation_.get_scene().terrains[0].width;
    for(const auto& u : updates) {
      for(int j = 0; j < u.height; j++) {
        for(int i = 0; i < u.width; i++) {
          uploaded[size_t((u.y + j) * width + u.x + i)] =
            u.z[size_t(j * u.width + i)];
        }
      }
    }
  }

  std::string path_;
  shift::gui::Simulation simulation_;
  shift::core::terrain::Scm* scm_ = nullptr;
};

static float MaxDifference(const std::vector<float>& a,
                           const std::vector<float>& b) {
  float d = 0.0f;
  for(size_t k = 0; k < a.size(); k++) {
    d = std::max(d, std::fabs(a[k] - b[k]));
  }
  return d;
}

TEST_F(ScmPlayback, ForwardAndBack) {
  std::vector<float> uploaded = simulation_.get_scene().terrains[0].z;
  RunToEnd();
  std::vector<float> end;
  scm_->GetLevels(end);
  const std::vector<float> zero(end.size(), 0.0f);
  ASSERT_GT(MaxDifference(end, zero), 0.01f);

  simulation_.ShowTime(1.0);
  EXPECT_EQ(Shown(), end);
  simulation_.ShowTime(0.0);
  EXPECT_EQ(Shown(), zero);
  /* part way, between the start and the end */
  simulation_.ShowTime(0.01);
  const std::vector<float> early = Shown();
  simulation_.ShowTime(0.5);
  simulation_.ShowTime(0.01);
  EXPECT_EQ(Shown(), early);
  float deepest = 0.0f;
  for(float level : early) {
    deepest = std::max(deepest, level);
  }
  EXPECT_GT(deepest, 0.0f);
  EXPECT_LT(deepest, *std::max_element(end.begin(), end.end()));
  simulation_.ShowTime(1.0);
  EXPECT_EQ(Shown(), end);
  /* the rectangles uploaded keep the texture as the scene */
  Upload(uploaded);
  EXPECT_EQ(uploaded, simulation_.get_scene().terrains[0].z);

  simulation_.Reset();
  EXPECT_EQ(Shown(), zero);
  EXPECT_EQ(scm_->get_changed_node_count(), 0u);
  Upload(uploaded);
  EXPECT_EQ(uploaded, simulation_.get_scene().terrains[0].z);
}

/* Run from a time shown starts the terrain from the levels shown */
TEST_F(ScmPlayback, RunFromShownLevels) {
  RunToEnd();
  simulation_.ShowTime(0.3);
  const std::vector<float> shown = Shown();
  ASSERT_TRUE(simulation_.AdoptShownState());
  std::vector<float> levels;
  scm_->GetLevels(levels);
  EXPECT_EQ(levels, shown);
  RunToEnd();
  /* the first knot point is the levels adopted */
  simulation_.ShowTime(0.0);
  EXPECT_EQ(Shown(), shown);
  simulation_.ShowTime(1.0);
  scm_->GetLevels(levels);
  EXPECT_EQ(Shown(), levels);
}

/* the model of the fixture logged to an HDF5 file and played back from the
log: the levels and the plate at the end are those of the run */
TEST_F(ScmPlayback, PlaysBackAnHdf5Log) {
  const std::string h5 = testing::TempDir() + "/scm_playback.h5";
  std::string model = kModel;
  model.replace(model.find(R"("integrators")"), 0,
                R"("loggers": [{"type": "hdf5", "logfile": ")" + h5 +
                  R"("}], )");
  const std::string path = testing::TempDir() + "/scm_logged.json";
  {
    std::ofstream file(path);
    file << model;
  }
  shift::core::Context context;
  context.LoadJSON(path);
  double plate = 0.0; // q of the slide at the last output
  context.set_state_observer(
    [&](double, const std::vector<double>& y) { plate = y[0]; });
  context.Simulate();
  auto* scm = dynamic_cast<shift::core::terrain::Scm*>(
    context.get_active_terrain()[0]);
  std::vector<float> end;
  scm->GetLevels(end);
  std::remove(path.c_str());

  shift::gui::Simulation log;
  log.Load(h5);
  ASSERT_TRUE(log.has_results());
  EXPECT_DOUBLE_EQ(log.get_computed_time(), 1.0);
  scm_ = dynamic_cast<shift::core::terrain::Scm*>(
    log.get_context()->get_active_terrain()[0]);
  log.ShowTime(1.0);
  EXPECT_DOUBLE_EQ(log.get_shown_q(0), plate);
  const auto& t = log.get_scene().terrains[0];
  float difference = 0.0f;
  for(long j = 0; j < scm_->get_node_count_y(); j++) {
    for(long i = 0; i < scm_->get_node_count_x(); i++) {
      const float level = t.z[size_t((j + 1) * t.width + i + 1)] -
                          float(scm_->UndisturbedZ(i, j));
      difference = std::max(
        difference,
        std::fabs(level - end[size_t(j * scm_->get_node_count_x() + i)]));
    }
  }
  EXPECT_EQ(difference, 0.0f);
  log.ShowTime(0.0);
  EXPECT_EQ(Shown().size(), end.size());
  std::remove(h5.c_str());
}

/* the double pendulum, with its mesh files in another directory than the model
(include_directories), logged and played back: the log holds the mesh and
its material file, and the viewer finds them, loads the model without its
loggers, so that a run from the log leaves the log as it was, and shows the
state of the run */
TEST(Hdf5Log, PlaysBackWithTheFilesOfTheModel) {
  const std::filesystem::path directory =
    std::filesystem::path(testing::TempDir()) / "hdf5_log" / "run";
  std::filesystem::create_directories(directory);
  const std::string h5 = (directory / "pendulum.h5").string();
  Json::Value model;
  {
    std::ifstream in(std::string(SHIFT_EXAMPLES_DIR) + "/pendulum/double_pendulum.json");
    in >> model;
  }
  Json::Value& parameters = model["model_parameters"];
  parameters["include_directories"] = Json::Value(Json::arrayValue);
  parameters["include_directories"].append(std::string(SHIFT_EXAMPLES_DIR) +
                                           "/pendulum");
  parameters["integrators"][0]["time_span"] = 1.0;
  parameters["loggers"] = Json::Value(Json::arrayValue);
  Json::Value logger;
  logger["type"] = "hdf5";
  logger["logfile"] = h5;
  parameters["loggers"].append(logger);
  const std::string path = (directory / "pendulum.json").string();
  {
    std::ofstream out(path);
    out << model;
  }
  shift::core::Context context;
  context.LoadJSON(path);
  bool obj = false, mtl = false;
  for(const auto& file : context.get_files_read()) {
    obj = obj || file.find("pendulum/link.obj") != std::string::npos;
    mtl = mtl || file.find("pendulum/link.mtl") != std::string::npos;
  }
  EXPECT_TRUE(obj);
  EXPECT_TRUE(mtl);
  std::vector<double> q; // at the last output
  const size_t q_count = context.get_q_count();
  context.set_state_observer([&](double, const std::vector<double>& y) {
    q.assign(y.begin(), y.begin() + q_count);
  });
  context.Simulate();
  std::filesystem::remove(path);

  shift::gui::Simulation log;
  log.Load(h5);
  ASSERT_TRUE(log.has_results());
  EXPECT_EQ(log.get_filename(), h5);
  const std::string loaded = log.get_context()->get_filename();
  EXPECT_EQ(loaded.find(directory.string()), std::string::npos) << loaded;
  EXPECT_FALSE(log.get_scene().meshes.empty());
  /* every file from the directory the log was taken out to */
  const std::string taken_out =
    (std::filesystem::temp_directory_path() / "shift-log-").string();
  EXPECT_EQ(loaded.rfind(taken_out, 0), 0u) << loaded;
  EXPECT_EQ(log.get_context()->get_files_read().size(), 2u);
  for(const auto& file : log.get_context()->get_files_read()) {
    EXPECT_EQ(file.rfind(taken_out, 0), 0u) << file;
  }
  EXPECT_TRUE(log.get_context()->get_active_loggers().empty());
  log.ShowTime(1.0);
  for(size_t k = 0; k < q.size(); k++) {
    EXPECT_DOUBLE_EQ(log.get_shown_q(int(k)), q[k]) << k;
  }
  const auto size = std::filesystem::file_size(h5);
  log.Run();
  while(log.is_computing()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_EQ(log.get_error(), "");
  EXPECT_EQ(std::filesystem::file_size(h5), size);
  std::filesystem::remove_all(directory.parent_path());
}
