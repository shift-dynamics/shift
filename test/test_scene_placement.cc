/* The placement of the viewer's instances (SceneInstance::placement): a
terrain's mesh is in the coordinates of the fixed body, unplaced, so that the
ray tracer, which places every instance, finds it where the rasterizer draws
it */

#include <gtest/gtest.h>

#include <string>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/gui/simulation.h"

TEST(ScenePlacement, TerrainIsNotPlaced) {
  shift::gui::Simulation simulation;
  simulation.Load(std::string(SHIFT_EXAMPLES_DIR) +
                  "/bricard/bricard_asphalt.json");
  const shift::gui::Scene& scene = simulation.get_scene();
  int terrains = 0;
  for(const auto& instance : scene.instances) {
    if(scene.meshes[size_t(instance.mesh_index)].terrain < 0) {
      continue;
    }
    terrains++;
    EXPECT_FALSE(instance.is_placed());
    EXPECT_EQ(instance.scale, 1.f);
    for(int r = 0; r < 3; r++) {
      for(int c = 0; c < 3; c++) {
        EXPECT_EQ(instance.placement.R.at[r][c], r == c ? 1.f : 0.f);
      }
    }
  }
  EXPECT_EQ(terrains, 1);
}
