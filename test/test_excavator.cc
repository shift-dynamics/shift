#include "gtest/gtest.h"
#include <filesystem>
#include <iostream>
#include <string>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"

/* run each test in the build tree so the model's logs stay out of the
source tree */
class Excavator : public testing::Test {
protected:
  void SetUp() override {
    const std::filesystem::path dir =
      std::filesystem::path(SHIFT_BINARY_DIR) / "test" / "excavator";
    std::filesystem::create_directories(dir);
    previous_dir_ = std::filesystem::current_path();
    std::filesystem::current_path(dir);
  }
  void TearDown() override {
    std::filesystem::current_path(previous_dir_);
  }

  std::filesystem::path previous_dir_;
};

TEST_F(Excavator, DiffLogs) {

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

TEST_F(Excavator, DISABLED_Graphviz) {

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
