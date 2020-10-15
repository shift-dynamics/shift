#include "gtest/gtest.h"
#include <iostream>
#include <string>

#include "shift/core/context.h"
#include "shift/config.h"

TEST(Excavator, DiffLogs) {

  std::string root_dir(SHIFT_ROOT_DIR);
  std::string json_file = 
    root_dir + "/external/models/excavator/excavator.json"; 
  std::cout << json_file << std::endl;

  shift::core::Context context;
  context.loadJSON(json_file);
  context.digraphFrames();
  context.simulate();

}
