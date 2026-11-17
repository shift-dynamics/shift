/* What a model reports while it loads (Context::set_load_observer), and a
load stopped by its observer */

#include <gtest/gtest.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include "shift/config.h"
#include "shift/core/context.h"

static const std::string kModel =
  std::string(SHIFT_EXAMPLES_DIR) + "/beach/strandbeest_beach.json";

/* the stages in order, the items of a counted stage from the first to the
last */
TEST(LoadProgress, Stages) {
  shift::core::Context context;
  std::vector<std::string> stages;
  int geometry_seen = 0, geometry_total = 0;
  context.set_load_observer(
    [&](const std::string& stage, int done, int total) {
      if(stages.empty() || stages.back() != stage) {
        stages.push_back(stage);
      }
      if(stage == "Loading geometry") {
        EXPECT_EQ(done, geometry_seen) << "items in order";
        EXPECT_LT(done, total);
        geometry_seen++;
        geometry_total = total;
      }
    });
  context.LoadJSON(kModel);
  ASSERT_FALSE(stages.empty());
  EXPECT_EQ(stages.front(), "Reading the model");
  EXPECT_EQ(stages.back(), "Setting up contact detection");
  EXPECT_GT(geometry_total, 0);
  EXPECT_EQ(geometry_seen, geometry_total);
  for(const char* stage : {"Setting up the topology", "Loading terrain",
                           "Placing scenery", "Fitting collision geometry"}) {
    EXPECT_NE(std::find(stages.begin(), stages.end(), stage), stages.end())
      << stage;
  }
}

/* an observer that throws stops the load there, and LoadJSON passes the
exception on */
TEST(LoadProgress, Stopped) {
  struct Stop : std::runtime_error {
    Stop()
      : std::runtime_error("stop") {}
  };
  std::vector<std::string> after;
  bool stopped = false;
  {
    shift::core::Context context;
    context.set_load_observer([&](const std::string& stage, int, int) {
      if(stopped) {
        after.push_back(stage);
      }
      if(stage == "Loading geometry") {
        stopped = true;
        throw Stop();
      }
    });
    EXPECT_THROW(context.LoadJSON(kModel), Stop);
  }
  EXPECT_TRUE(stopped);
  EXPECT_TRUE(after.empty()) << "nothing reported after the stop";
}
