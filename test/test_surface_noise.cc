/* the noise levels of the surfaces drawn over a terrain in patches
(shift::gui::SurfaceNoiseLevel) */

#include <gtest/gtest.h>

#include "shift/gui/scene.h"

/* more coverage, a lower level; none and all are beyond the noise (0 to 1),
and half is near its middle */
TEST(SurfaceNoise, LevelsFallWithCoverage) {
  for(int surface : {0, 1}) {
    EXPECT_GT(shift::gui::SurfaceNoiseLevel(surface, 0.0), 1.f);
    EXPECT_LT(shift::gui::SurfaceNoiseLevel(surface, 1.0), 0.f);
    float previous = 1.f;
    for(double coverage = 0.05; coverage < 1.0; coverage += 0.1) {
      const float level = shift::gui::SurfaceNoiseLevel(surface, coverage);
      EXPECT_LT(level, previous) << coverage;
      EXPECT_GT(level, 0.f);
      previous = level;
    }
    EXPECT_NEAR(shift::gui::SurfaceNoiseLevel(surface, 0.5), 0.5f, 0.1f);
  }
}
