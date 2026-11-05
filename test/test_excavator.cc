#include "gtest/gtest.h"
#include <iostream>
#include <string>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"

TEST(Excavator, DiffLogs) {

  try {
    std::string examples_dir(SHIFT_EXAMPLES_DIR);
    std::string json_file = examples_dir + "/excavator/excavator.json";
    shift::core::Context context;

    context.LoadJSON(json_file);
    context.Simulate();

  } catch(const shift::core::Exception& e) {
    std::cout << std::endl
              << "Caught exception: `" << e.what() << "`" << std::endl;
    std::cout << e.backtrace() << std::endl;
    FAIL();
  }

  SUCCEED();
}

TEST(Excavator, DISABLED_Graphviz) {

  try {
    std::string examples_dir(SHIFT_EXAMPLES_DIR);
    std::string json_file = examples_dir + "/excavator/excavator_no_logging.json";
    shift::core::Context context;

    context.LoadJSON(json_file);
    context.Digraph("excavator_digraph.png");

  } catch(const shift::core::Exception& e) {
    std::cout << std::endl
              << "Caught exception: `" << e.what() << "`" << std::endl;
    std::cout << e.backtrace() << std::endl;
    FAIL();
  }

  SUCCEED();
}
