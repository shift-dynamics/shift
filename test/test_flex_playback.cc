/* the viewer's drawing of a flexible body built from a mesh: the steel
plate of examples/cad/plate_hole.step on a pin at its bore, a modal body
with an initial deformation. Its mesh surface's vertices follow the elastic
coordinates, the deformation scale magnifying the part of the deformation
that is not a rigid motion, and a mode previewed moves the surface by the
amplitude asked. The CAD solid deforms with the mesh: its samples move as
the mesh's surface around them. */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/gui/simulation.h"

#if SHIFT_HAVE_FEA && SHIFT_HAVE_STEP

static std::string WritePlate(double time_step = 0.001) {
  const std::string path = testing::TempDir() + "/flex_playback.json";
  std::ofstream file(path);
  file << R"({"model_parameters": {"version": "0.2",
      "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                "time": "seconds"},
      "gravity": [0, 9.81, 0], "solvers": [{"type": "rnca"}],
      "include_directories": [")"
       << SHIFT_EXAMPLES_DIR << R"(/cad"],
      "integrators": [{"type": "ode113", "time_span": 0.001,
                       "time_step": )" << time_step << R"(, "hmax": 1e-4}]},
      "assemblies": [{"name": "test", "bodies": [{"name": "plate",
        "type": "modal", "mode_count": 2,
        "initial_elastic_coordinates": [1e-4, 0],
        "density": 7850, "youngs_modulus": 2.1e11, "poisson_ratio": 0.3,
        "fe_geometry": "solid", "element_size": 0.02,
        "frames": [{"name": "pin", "T": [0, 0, 0.01]}],
        "geometry": [{"name": "solid", "type": "cad",
                      "filename": "plate_hole.step"}]}],
      "joints": [{"name": "hinge", "type": "revolute",
                  "parent": [["fixed"], "origin"],
                  "child": [["plate"], "pin"], "axis": [0, 0, 1]}]}]})";
  return path;
}

/* the largest distance between the vertices of two surfaces */
static double LargestMove(const std::vector<shift::gui::Vertex>& a,
                          const std::vector<shift::gui::Vertex>& b) {
  double largest = 0.0;
  for(size_t v = 0; v < a.size(); v++) {
    double d2 = 0.0;
    for(int c = 0; c < 3; c++) {
      const double d = a[v].position[c] - b[v].position[c];
      d2 += d * d;
    }
    largest = std::max(largest, std::sqrt(d2));
  }
  return largest;
}

/* the surface an update draws */
static std::vector<shift::gui::Vertex>
Deformed(const shift::gui::Scene& scene,
         const shift::gui::FlexUpdate& update) {
  std::vector<shift::gui::Vertex> vertices;
  shift::gui::DeformFlexMesh(scene.meshes[update.mesh], update, vertices);
  return vertices;
}

TEST(FlexPlayback, DrawsTheDeformedMeshSurface) {
  const std::string path = WritePlate();
  shift::gui::Simulation simulation;
  const std::string cache =
    testing::TempDir() + "/flex_playback.test__plate.fe.h5";
  std::remove(cache.c_str());
  /* the viewer is told what is being done, the flexible body's stages
  among them; stopped by the observer, no model is loaded */
  std::vector<std::string> stages;
  simulation.set_load_observer(
    [&stages](const std::string& stage, int, int) {
      if(stages.empty() || stages.back() != stage) {
        stages.push_back(stage);
      }
    });
  simulation.Load(path);
  auto reported = [&stages](const std::string& part) {
    return std::any_of(stages.begin(), stages.end(),
                       [&](const std::string& s) {
                         return s.find(part) != std::string::npos;
                       });
  };
  EXPECT_TRUE(reported("'plate': meshing its solid"));
  EXPECT_TRUE(reported("'plate': computing its modes"));
  EXPECT_TRUE(reported("'plate': sampling its CAD"));
  {
    shift::gui::Simulation stopped;
    stopped.set_load_observer([](const std::string& stage, int, int) {
      if(stage.find("sampling its CAD") != std::string::npos) {
        throw std::runtime_error("cancelled");
      }
    });
    EXPECT_THROW(stopped.Load(path), std::runtime_error);
    EXPECT_FALSE(stopped.is_loaded());
  }
  /* the nodes the pin's attachment ties, on the surface, are drawn as
  points: the same read from the cache as computed */
  std::vector<uint32_t> constrained;
  for(const auto& mesh : simulation.get_scene().meshes) {
    if(mesh.deformable) {
      constrained = mesh.constrained_vertices;
    }
  }
  EXPECT_FALSE(constrained.empty());
  {
    shift::gui::Simulation cached;
    cached.Load(path);
    for(const auto& mesh : cached.get_scene().meshes) {
      if(mesh.deformable) {
        EXPECT_EQ(mesh.constrained_vertices, constrained);
      }
    }
  }
  simulation.set_load_observer(nullptr);
  std::remove(path.c_str());
  std::remove(cache.c_str());
  ASSERT_EQ(simulation.get_flex_bodies().size(), 1u);
  const int body = simulation.get_flex_bodies()[0];
  const shift::gui::Scene& scene = simulation.get_scene();
  int deformable = -1;
  for(size_t m = 0; m < scene.meshes.size(); m++) {
    if(scene.meshes[m].deformable) {
      deformable = static_cast<int>(m);
    }
  }
  ASSERT_GE(deformable, 0);
  EXPECT_FALSE(scene.meshes[deformable].indices.empty());

  /* the solid's CAD, of its own, deformed with the mesh: samples of each
  patch, and their shapes */
  ASSERT_EQ(scene.cads.size(), 1u);
  const shift::gui::SceneCad& cad = scene.cads[0];
  EXPECT_EQ(cad.flex_mesh, deformable);
  ASSERT_EQ(cad.sample_grids.size(), cad.patches.size());
  const size_t samples = cad.sample_points.size() / 4;
  EXPECT_GT(samples, 0u);
  EXPECT_EQ(cad.sample_basis.size(),
            3 * scene.meshes[deformable].flex_columns * samples);
  for(size_t p = 0; p < cad.patches.size(); p++) {
    /* the ray tracer's triangles at least as fine as the samples */
    EXPECT_GE(cad.grids[2 * p], cad.sample_grids[p].grid[0]);
    EXPECT_GE(cad.grids[2 * p + 1], cad.sample_grids[p].grid[1]);
  }

  /* the scene keeps the undeformed surface, and the shapes of each vertex:
  the elastic coordinates', then the two modes computed */
  const shift::gui::SceneMesh& mesh = scene.meshes[deformable];
  const std::vector<shift::gui::Vertex>& undeformed = mesh.vertices;
  const unsigned int elastic = 2; /* the pin holds the plate: no static modes */
  EXPECT_EQ(mesh.flex_columns, elastic + 2u);
  EXPECT_EQ(mesh.flex_basis.size(), 3 * mesh.flex_columns * undeformed.size());

  /* the initial deformation as computed; then its part that is not a rigid
  motion of the body (the difference from scale 0) 1000 times as large */
  std::vector<shift::gui::FlexUpdate> updates;
  simulation.TakeFlexUpdates(updates, true);
  ASSERT_EQ(updates.size(), 1u);
  EXPECT_EQ(updates[0].mesh, deformable);
  ASSERT_EQ(updates[0].coefficients.size(), mesh.flex_columns);
  const std::vector<shift::gui::Vertex> deformed = Deformed(scene, updates[0]);
  EXPECT_GT(LargestMove(deformed, undeformed), 0.0);
  simulation.set_deformation_scale(0.0);
  simulation.TakeFlexUpdates(updates);
  ASSERT_EQ(updates.size(), 1u);
  const std::vector<shift::gui::Vertex> rigid = Deformed(scene, updates[0]);
  const double move = LargestMove(deformed, rigid);
  EXPECT_GT(move, 0.0);
  simulation.set_deformation_scale(1000.0);
  simulation.TakeFlexUpdates(updates);
  ASSERT_EQ(updates.size(), 1u);
  /* the vertices are floats */
  EXPECT_NEAR(LargestMove(Deformed(scene, updates[0]), rigid) / move, 1000.0,
              0.1);
  /* nothing changed: nothing to upload */
  simulation.TakeFlexUpdates(updates);
  EXPECT_TRUE(updates.empty());

  /* the second mode computed, its largest displacement 5 mm (over all the
  nodes; the surface's is at most that) */
  simulation.PreviewShape(body, 1, 1, 0.005);
  simulation.TakeFlexUpdates(updates);
  ASSERT_EQ(updates.size(), 1u);
  const std::vector<shift::gui::Vertex> mode = Deformed(scene, updates[0]);
  const double preview = LargestMove(mode, undeformed);
  EXPECT_LE(preview, 0.005 * (1.0 + 1e-5));
  EXPECT_GT(preview, 0.004);

  /* the CAD's samples move as the nearest vertex of the mesh's surface, to
  the change of the mode over the distance between them (a node spacing at
  most); interpolated over a patch, the field is the samples' at them */
  std::vector<float> field;
  shift::gui::FlexCadField(cad, mesh, updates[0], field);
  ASSERT_EQ(field.size(), cad.sample_points.size());
  double largest_difference = 0.0;
  for(size_t k = 0; k < samples; k++) {
    const float* x = cad.sample_points.data() + 4 * k;
    size_t nearest = 0;
    double best = 1e300;
    for(size_t v = 0; v < undeformed.size(); v++) {
      double d2 = 0.0;
      for(int c = 0; c < 3; c++) {
        const double d = undeformed[v].position[c] - x[c];
        d2 += d * d;
      }
      if(d2 < best) {
        best = d2;
        nearest = v;
      }
    }
    double d2 = 0.0;
    for(int c = 0; c < 3; c++) {
      const double d = field[4 * k + size_t(c)] -
                       (mode[nearest].position[c] -
                        undeformed[nearest].position[c]);
      d2 += d * d;
    }
    largest_difference = std::max(largest_difference, std::sqrt(d2));
  }
  EXPECT_LT(largest_difference, 0.15 * preview);
  for(size_t p = 0; p < cad.sample_grids.size(); p++) {
    const auto& g = cad.sample_grids[p];
    for(int j = 0; j <= g.grid[1]; j += std::max(1, g.grid[1])) {
      for(int i = 0; i <= g.grid[0]; i += std::max(1, g.grid[0])) {
        const float u = i == g.grid[0] ? g.uv[2] : g.uv[0];
        const float v = j == g.grid[1] ? g.uv[3] : g.uv[1];
        const ksl_vec3f_t d = shift::gui::FlexCadInterpolate(
          cad, field, static_cast<int>(p), u, v, nullptr, nullptr);
        const size_t k =
          size_t(g.grid[2]) + size_t(j) * size_t(g.grid[0] + 1) + size_t(i);
        for(int c = 0; c < 3; c++) {
          EXPECT_NEAR(d.at[c], field[4 * k + size_t(c)], 1e-9);
        }
      }
    }
  }
  simulation.ClearShapePreview();
  simulation.set_deformation_scale(1.0);
  simulation.TakeFlexUpdates(updates);
  ASSERT_EQ(updates.size(), 1u);
  EXPECT_LT(LargestMove(Deformed(scene, updates[0]), deformed), 1e-9);
}

/* the principal stresses in closed form: of R diag(300, 100, -200) R^T,
and of a tensor with a repeated one */
TEST(FlexStress, PrincipalStresses) {
  const double r[3][3] = {{2.0 / 3, -1.0 / 3, 2.0 / 3},
                          {2.0 / 3, 2.0 / 3, -1.0 / 3},
                          {-1.0 / 3, 2.0 / 3, 2.0 / 3}};
  const double eigen[3] = {300.0, 100.0, -200.0};
  double a[3][3] = {};
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      for(int k = 0; k < 3; k++) {
        a[i][j] += r[i][k] * eigen[k] * r[j][k];
      }
    }
  }
  const float s[6] = {float(a[0][0]), float(a[1][1]), float(a[2][2]),
                      float(a[0][1]), float(a[1][2]), float(a[2][0])};
  float principal[3];
  shift::gui::FlexPrincipalStresses(s, principal);
  for(int k = 0; k < 3; k++) {
    EXPECT_NEAR(principal[k], eigen[k], 1e-3);
  }
  EXPECT_NEAR(shift::gui::FlexStressValue(
                s, shift::gui::FlexUpdate::Quantity::kTresca),
              500.0, 1e-3);
  /* von Mises from the principal stresses */
  EXPECT_NEAR(shift::gui::FlexStressValue(
                s, shift::gui::FlexUpdate::Quantity::kVonMises),
              std::sqrt(0.5 * (200.0 * 200.0 + 300.0 * 300.0 +
                               500.0 * 500.0)),
              1e-3);
  EXPECT_FLOAT_EQ(shift::gui::FlexStressValue(
                    s, shift::gui::FlexUpdate::Quantity::kYz),
                  s[4]);
  const float uniaxial[6] = {0.f, 0.f, -7.f, 0.f, 0.f, 0.f};
  shift::gui::FlexPrincipalStresses(uniaxial, principal);
  /* at a repeated eigenvalue the closed form loses half the digits: to
  about sqrt(float epsilon) of the stress */
  const float tolerance = 7.f * 2e-4f;
  EXPECT_NEAR(principal[0], 0.f, tolerance);
  EXPECT_NEAR(principal[1], 0.f, tolerance);
  EXPECT_NEAR(principal[2], -7.f, tolerance);
}

/* the greatest stress over the run: the elastic coordinates of the states
recorded up to the time shown, from the first again when the time goes
back, and of those not sent yet when it goes on; the safety factor drawn
once the body's yield_strength, a key that may change as the model runs,
is set */
TEST(FlexPlayback, DrawsTheGreatestStressOverTheRun) {
  const std::string path = WritePlate(1e-4);
  const std::string cache =
    testing::TempDir() + "/flex_playback.test__plate.fe.h5";
  std::remove(cache.c_str());
  shift::gui::Simulation simulation;
  simulation.Load(path);
  std::remove(path.c_str());
  std::remove(cache.c_str());
  const int body = simulation.get_flex_bodies().at(0);
  const size_t n = simulation.get_bodies()[size_t(body)]->get_elastic_count();
  simulation.Run();
  while(simulation.is_computing()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(simulation.get_error().empty()) << simulation.get_error();
  const double end = simulation.get_computed_time();
  ASSERT_GT(end, 0.0);
  std::vector<bool> stressed(simulation.get_bodies().size(), false);
  stressed[size_t(body)] = true;
  simulation.set_flex_stresses(stressed, true, 0.0, 0.0);
  std::vector<shift::gui::FlexUpdate> updates;
  /* the states at the start and the end */
  simulation.ShowTime(0.0);
  simulation.TakeFlexUpdates(updates, true);
  ASSERT_EQ(updates.size(), 1u);
  EXPECT_FALSE(updates[0].peak);
  EXPECT_TRUE(updates[0].peak_samples.empty());
  const std::vector<float> first(updates[0].stress_coefficients.begin(),
                                 updates[0].stress_coefficients.begin() + n);
  simulation.ShowTime(end);
  simulation.TakeFlexUpdates(updates);
  ASSERT_EQ(updates.size(), 1u);
  const std::vector<float> last(updates[0].stress_coefficients.begin(),
                                updates[0].stress_coefficients.begin() + n);

  /* all of them at the end */
  simulation.set_flex_stress_peak(true);
  simulation.TakeFlexUpdates(updates);
  ASSERT_EQ(updates.size(), 1u);
  EXPECT_TRUE(updates[0].peak);
  EXPECT_EQ(updates[0].peak_first, 0u);
  EXPECT_EQ(updates[0].peak_columns, n);
  const std::vector<float> all = updates[0].peak_samples;
  ASSERT_EQ(all.size() % n, 0u);
  const size_t states = all.size() / n;
  ASSERT_GE(states, 5u);
  for(size_t j = 0; j < n; j++) {
    EXPECT_EQ(all[j], first[j]);
    EXPECT_EQ(all[(states - 1) * n + j], last[j]);
  }
  /* drawn: at least the stress now everywhere, and somewhere more */
  const shift::gui::SceneMesh& mesh =
    simulation.get_scene().meshes[size_t(updates[0].mesh)];
  std::vector<float> peak;
  std::vector<shift::gui::Vertex> vertices;
  shift::gui::DeformFlexMesh(mesh, updates[0], vertices, &peak);
  ASSERT_EQ(peak.size(), mesh.vertices.size());
  bool greater = false;
  for(size_t v = 0; v < peak.size(); v++) {
    const float now = shift::gui::FlexStressAt(
      mesh, last.data(), n, v, shift::gui::FlexUpdate::Quantity::kVonMises);
    EXPECT_GE(peak[v], now * (1.f - 1e-6f));
    greater = greater || peak[v] > 1.01f * now;
  }
  EXPECT_TRUE(greater);
  /* back: from the first again, fewer of them */
  simulation.ShowTime(0.5 * end);
  simulation.TakeFlexUpdates(updates);
  ASSERT_EQ(updates.size(), 1u);
  EXPECT_EQ(updates[0].peak_first, 0u);
  const size_t half = updates[0].peak_samples.size() / n;
  ASSERT_GT(half, 0u);
  ASSERT_LT(half, states);
  for(size_t i = 0; i < half * n; i++) {
    EXPECT_EQ(updates[0].peak_samples[i], all[i]);
  }
  /* on: those after them */
  simulation.ShowTime(end);
  simulation.TakeFlexUpdates(updates);
  ASSERT_EQ(updates.size(), 1u);
  EXPECT_EQ(updates[0].peak_first, half);
  ASSERT_EQ(updates[0].peak_samples.size(), (states - half) * n);
  for(size_t i = 0; i < updates[0].peak_samples.size(); i++) {
    EXPECT_EQ(updates[0].peak_samples[i], all[half * n + i]);
  }
  /* nothing changed: nothing sent */
  simulation.TakeFlexUpdates(updates);
  EXPECT_TRUE(updates.empty());

  /* the safety factor: none without a yield strength, then one */
  simulation.set_flex_stress_quantity(
    shift::gui::FlexUpdate::Quantity::kSafetyFactor, 1.0, 4.0);
  simulation.TakeFlexUpdates(updates);
  ASSERT_EQ(updates.size(), 1u);
  EXPECT_EQ(updates[0].stress, shift::gui::FlexUpdate::Stress::kNone);
  shift::core::parameter::Value* yield =
    simulation.get_bodies()[size_t(body)]->Find("yield_strength");
  ASSERT_NE(yield, nullptr);
  EXPECT_TRUE(yield->is_runtime());
  Json::Value strength(2.5e8);
  yield->ReadJSON(strength);
  simulation.TakeFlexUpdates(updates);
  ASSERT_EQ(updates.size(), 1u);
  EXPECT_EQ(updates[0].stress, shift::gui::FlexUpdate::Stress::kRange);
  EXPECT_EQ(updates[0].quantity,
            shift::gui::FlexUpdate::Quantity::kSafetyFactor);
  EXPECT_FLOAT_EQ(updates[0].yield_strength, 2.5e8f);
  EXPECT_FLOAT_EQ(updates[0].stress_range[0], 1.f);
  EXPECT_FLOAT_EQ(updates[0].stress_range[1], 4.f);
  /* the peak starts again with the quantity */
  EXPECT_EQ(updates[0].peak_first, 0u);
  EXPECT_EQ(updates[0].peak_samples, all);
}

#endif
