/* modal and nodal bodies built from a STEP solid in a model file: the steel
plate of examples/cad/plate_hole.step (80 x 80 x 20 mm, a bore of radius
10.5 mm on the z axis) hangs on a revolute joint at its bore. The bore's
faces are found from the joint, the mesh is reduced, the mass is the
plate's, and the body swings briefly under gravity. Frames inside the
material are mesh nodes. */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/config.h"
#include "shift/core/body/flex.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"

#if SHIFT_HAVE_FEA && SHIFT_HAVE_STEP

/* the cache of a test's model, which each test starts without */
static std::string CachePath(const std::string& name) {
  return testing::TempDir() + "/" + name + ".test__plate.fe.h5";
}

static void LoadPlate(shift::core::Context& context, const std::string& name,
                      const std::string& body_keys, bool keep_cache = false,
                      double element_size = 0.02) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  if(!keep_cache) {
    std::remove(CachePath(name).c_str());
  }
  {
    std::ofstream file(path);
    file << R"({"model_parameters": {"version": "0.2",
      "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                "time": "seconds"},
      "gravity": [0, 9.81, 0], "solvers": [{"type": "rnca"}],
      "include_directories": [")"
         << SHIFT_EXAMPLES_DIR << R"(/cad"],
      "integrators": [{"type": "ode113", "time_span": 0.002,
                       "time_step": 0.001, "hmax": 1e-4,
                       "relative_error": 1e-8, "absolute_error": 1e-10}]},
      "assemblies": [{"name": "test", "bodies": [{"name": "plate",
        "density": 7850, "youngs_modulus": 2.1e11, "poisson_ratio": 0.3,
        "fe_geometry": "solid", "element_size": )"
         << element_size << R"(,
        "frames": [{"name": "pin", "T": [0, 0, 0.01]},
                   {"name": "corner", "T": [0.035, 0.035, 0.01]}],
        "geometry": [{"name": "solid", "type": "cad",
                      "filename": "plate_hole.step"}],
        )" << body_keys
         << R"(}],
      "joints": [{"name": "hinge", "type": "revolute",
                  "parent": [["fixed"], "origin"],
                  "child": [["plate"], "pin"], "axis": [0, 0, 1]}]}]})";
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
  if(!keep_cache) {
    std::remove(CachePath(name).c_str());
  }
}

static shift::core::body::Flex* Plate(shift::core::Context& context) {
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "plate") {
      return dynamic_cast<shift::core::body::Flex*>(body);
    }
  }
  return nullptr;
}

static double PlateMass() {
  return 7850.0 * (0.08 * 0.08 * 0.02 - M_PI * 0.0105 * 0.0105 * 0.02);
}

TEST(FlexStep, ModalPlateOnItsBore) {
  shift::core::Context context;
  LoadPlate(context, "modal_plate", R"("type": "modal", "mode_count": 3,
                                       "damping_ratio": 0.02)");
  shift::core::body::Flex* plate = Plate(context);
  ASSERT_NE(plate, nullptr);
  /* one attachment (the bore, the reference): no static modes */
  EXPECT_EQ(plate->get_static_count(), 0u);
  EXPECT_EQ(plate->get_elastic_count(), 3u);
  ASSERT_EQ(plate->get_frequencies().size(), 3u);
  EXPECT_GT(plate->get_frequencies()[0], 1000.0);
  EXPECT_LE(plate->get_frequencies()[0], plate->get_frequencies()[1]);
  EXPECT_FALSE(plate->get_mesh_boundary().empty());

  EXPECT_NEAR(plate->get_inertia_wrt_reference().m / PlateMass(), 1.0, 2e-3);

  std::vector<double> last;
  context.set_state_observer(
    [&last](double, const std::vector<double>& y) { last = y; });
  context.Simulate();
  ASSERT_FALSE(last.empty());
  for(double y : last) {
    EXPECT_TRUE(std::isfinite(y));
  }
}

/* the corner frame, used by nothing, is tied only because attachments
lists it: its six motions and the three of each retained point (a nested
list) are the elastic coordinates */
TEST(FlexStep, NodalPlateWithAnAttachment) {
  shift::core::Context context;
  LoadPlate(context, "nodal_plate", R"("type": "nodal",
    "attachments": [{"frame": "corner", "radius": 0.012}],
    "retained_points": [[-0.03, -0.03, 0.01], [0.03, -0.03, 0.01]])");
  shift::core::body::Flex* plate = Plate(context);
  ASSERT_NE(plate, nullptr);
  EXPECT_EQ(plate->get_static_count(), 12u);
  EXPECT_EQ(plate->get_elastic_count(), 12u);
  EXPECT_TRUE(plate->get_frequencies().empty());
  EXPECT_EQ(plate->get_node_count(), 2u);
}

/* a frame inside the material and a retained point are nodes of the mesh,
exactly where they are; the frame is tied to its node and the nodes of the
elements around it. The pin's frame, in the bore, is not. */
TEST(FlexStep, FramesInsideTheMaterialAreNodes) {
  shift::core::Context context;
  LoadPlate(context, "interior_nodes", R"("type": "nodal",
    "attachments": [{"frame": "corner"}],
    "retained_points": [[-0.03, -0.03, 0.01]])");
  shift::core::body::Flex* plate = Plate(context);
  ASSERT_NE(plate, nullptr);
  EXPECT_EQ(plate->get_static_count(), 9u);
  const auto& positions = plate->get_positions();
  const unsigned int nodes = plate->get_mesh_node_count();
  auto node_at = [&](double x, double y, double z) {
    for(unsigned int i = 0; i < nodes; i++) {
      if(std::fabs(positions[i].x - x) < 1e-12 &&
         std::fabs(positions[i].y - y) < 1e-12 &&
         std::fabs(positions[i].z - z) < 1e-12) {
        return static_cast<int>(i);
      }
    }
    return -1;
  };
  const int corner = node_at(0.035, 0.035, 0.01);
  ASSERT_GE(corner, 0);
  EXPECT_GE(node_at(-0.03, -0.03, 0.01), 0);
  EXPECT_LT(node_at(0.0, 0.0, 0.01), 0);
  /* the attachments: the pin's (the reference), then the corner's */
  const auto& tied = plate->get_attachment_nodes();
  ASSERT_EQ(tied.size(), 2u);
  const std::vector<int>& ring = tied[1];
  EXPECT_NE(std::find(ring.begin(), ring.end(), corner), ring.end());
  EXPECT_GT(ring.size(), 10u);
  /* the ring: no farther than about an element from the frame */
  for(int i : ring) {
    const double dx = positions[size_t(i)].x - 0.035;
    const double dy = positions[size_t(i)].y - 0.035;
    const double dz = positions[size_t(i)].z - 0.01;
    EXPECT_LT(std::sqrt(dx * dx + dy * dy + dz * dz), 0.03);
  }
}

TEST(FlexStep, RefusesTwoModeSelections) {
  shift::core::Context context;
  EXPECT_THROW(LoadPlate(context, "two_selections",
                         R"("type": "modal", "mode_count": 3,
                            "modes_kept": [0, 1])"),
               shift::core::Exception);
}

/* the analysis is kept beside the model: loaded again, the body is read
from it, with the same matrices; another choice of the modes computed comes
from it too, and another mesh size runs the analysis again */
TEST(FlexStep, CachesTheAnalysis) {
  const std::string name = "cached_plate";
  std::remove(CachePath(name).c_str());
  const std::string keys = R"("type": "modal", "mode_count": 3)";
  shift::core::Context first;
  LoadPlate(first, name, keys, true);
  ASSERT_FALSE(Plate(first)->from_cache());
  shift::core::Context second;
  LoadPlate(second, name, keys, true);
  EXPECT_TRUE(Plate(second)->from_cache());
  EXPECT_EQ(Plate(second)->get_stiffness(), Plate(first)->get_stiffness());
  EXPECT_EQ(Plate(second)->get_frequencies(), Plate(first)->get_frequencies());
  EXPECT_EQ(Plate(second)->get_mesh_boundary(),
            Plate(first)->get_mesh_boundary());

  /* modes 0 and 2 of the 3: K^eta is their part of the first one's */
  shift::core::Context subset;
  LoadPlate(subset, name, R"("type": "modal", "modes_kept": [0, 2])", true);
  EXPECT_TRUE(Plate(subset)->from_cache());
  ASSERT_EQ(Plate(subset)->get_elastic_count(), 2u);
  const auto& k3 = Plate(first)->get_stiffness();
  const auto& k2 = Plate(subset)->get_stiffness();
  EXPECT_EQ(k2[0], k3[0]);
  EXPECT_EQ(k2[3], k3[8]);

  /* more modes than the cache holds, or another mesh: analysed again */
  shift::core::Context more;
  LoadPlate(more, name, R"("type": "modal", "mode_count": 4)", true);
  EXPECT_FALSE(Plate(more)->from_cache());
  shift::core::Context coarser;
  LoadPlate(coarser, name, keys, true, 0.03);
  EXPECT_FALSE(Plate(coarser)->from_cache());
  std::remove(CachePath(name).c_str());
}

/* the rows of a nested list must have three numbers each */
TEST(FlexStep, RefusesRaggedRetainedPoints) {
  shift::core::Context context;
  EXPECT_THROW(LoadPlate(context, "ragged",
                         R"("type": "nodal",
                            "retained_points": [[0.03, -0.03, 0.01],
                                                [0.03, -0.03]])"),
               shift::core::Exception);
}

#endif
