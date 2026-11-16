/* the viewer's drawing of a flexible body built from a mesh: the steel
plate of examples/cad/plate_hole.step on a pin at its bore, a modal body
with an initial deformation. Its mesh surface replaces the CAD solid, its
vertices follow the elastic coordinates, the deformation scale magnifying
the part of the deformation that is not a rigid motion, and a mode
previewed moves the surface by the amplitude asked. */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/gui/simulation.h"

#if SHIFT_HAVE_FEA && SHIFT_HAVE_STEP

static std::string WritePlate() {
  const std::string path = testing::TempDir() + "/flex_playback.json";
  std::ofstream file(path);
  file << R"({"model_parameters": {"version": "0.2",
      "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                "time": "seconds"},
      "gravity": [0, 9.81, 0], "solvers": [{"type": "rnca"}],
      "include_directories": [")"
       << SHIFT_EXAMPLES_DIR << R"(/cad"],
      "integrators": [{"type": "ode113", "time_span": 0.001,
                       "time_step": 0.001, "hmax": 1e-4}]},
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

/* the largest distance between the vertices of two updates */
static double LargestMove(const shift::gui::FlexUpdate& a,
                          const shift::gui::FlexUpdate& b) {
  double largest = 0.0;
  for(size_t v = 0; v < a.vertices.size(); v++) {
    double d2 = 0.0;
    for(int c = 0; c < 3; c++) {
      const double d = a.vertices[v].position[c] - b.vertices[v].position[c];
      d2 += d * d;
    }
    largest = std::max(largest, std::sqrt(d2));
  }
  return largest;
}

TEST(FlexPlayback, DrawsTheDeformedMeshSurface) {
  const std::string path = WritePlate();
  shift::gui::Simulation simulation;
  const std::string cache =
    testing::TempDir() + "/flex_playback.test__plate.fe.h5";
  std::remove(cache.c_str());
  simulation.Load(path);
  std::remove(path.c_str());
  std::remove(cache.c_str());
  ASSERT_EQ(simulation.get_flex_bodies().size(), 1u);
  const int body = simulation.get_flex_bodies()[0];
  const shift::gui::Scene& scene = simulation.get_scene();
  /* the mesh's surface, not the solid's CAD */
  EXPECT_TRUE(scene.cads.empty());
  int deformable = -1;
  for(size_t m = 0; m < scene.meshes.size(); m++) {
    if(scene.meshes[m].deformable) {
      deformable = static_cast<int>(m);
    }
  }
  ASSERT_GE(deformable, 0);
  EXPECT_FALSE(scene.meshes[deformable].indices.empty());

  /* the scene keeps the undeformed surface */
  shift::gui::FlexUpdate undeformed;
  undeformed.vertices = scene.meshes[deformable].vertices;

  /* the initial deformation as computed; then its part that is not a rigid
  motion of the body (the difference from scale 0) 1000 times as large */
  std::vector<shift::gui::FlexUpdate> updates;
  simulation.TakeFlexUpdates(updates, true);
  ASSERT_EQ(updates.size(), 1u);
  EXPECT_EQ(updates[0].mesh, deformable);
  const shift::gui::FlexUpdate deformed = updates[0];
  EXPECT_GT(LargestMove(deformed, undeformed), 0.0);
  simulation.set_deformation_scale(0.0);
  simulation.TakeFlexUpdates(updates);
  ASSERT_EQ(updates.size(), 1u);
  const shift::gui::FlexUpdate rigid = updates[0];
  const double move = LargestMove(deformed, rigid);
  EXPECT_GT(move, 0.0);
  simulation.set_deformation_scale(1000.0);
  simulation.TakeFlexUpdates(updates);
  ASSERT_EQ(updates.size(), 1u);
  /* the vertices are floats */
  EXPECT_NEAR(LargestMove(updates[0], rigid) / move, 1000.0, 0.1);
  /* nothing changed: nothing to upload */
  simulation.TakeFlexUpdates(updates);
  EXPECT_TRUE(updates.empty());

  /* the second mode computed, its largest displacement 5 mm (over all the
  nodes; the surface's is at most that) */
  simulation.PreviewShape(body, 1, 1, 0.005);
  simulation.TakeFlexUpdates(updates);
  ASSERT_EQ(updates.size(), 1u);
  const double preview = LargestMove(updates[0], undeformed);
  EXPECT_LE(preview, 0.005 * (1.0 + 1e-5));
  EXPECT_GT(preview, 0.004);
  simulation.ClearShapePreview();
  simulation.set_deformation_scale(1.0);
  simulation.TakeFlexUpdates(updates);
  ASSERT_EQ(updates.size(), 1u);
  EXPECT_LT(LargestMove(updates[0], deformed), 1e-9);
}

#endif
