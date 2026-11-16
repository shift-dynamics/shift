/* modal and nodal bodies built from a STEP solid in a model file: the steel
plate of examples/cad/plate_hole.step (80 x 80 x 20 mm, a bore of radius
10.5 mm on the z axis) hangs on a revolute joint at its bore. The bore's
faces are found from the joint, the mesh is reduced, the mass is the
plate's, and the body swings briefly under gravity. */

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

static void LoadPlate(shift::core::Context& context, const std::string& name,
                      const std::string& body_keys) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
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
        "fe_geometry": "solid", "element_size": 0.02,
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

TEST(FlexStep, RefusesTwoModeSelections) {
  shift::core::Context context;
  EXPECT_THROW(LoadPlate(context, "two_selections",
                         R"("type": "modal", "mode_count": 3,
                            "modes_kept": [0, 1])"),
               shift::core::Exception);
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
