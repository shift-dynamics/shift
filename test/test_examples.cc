/*
 * Loads and simulates every model in examples/.
 *
 * The models are discovered at run time; each one is a separate test case
 * named after its path relative to examples/ (non-alphanumeric characters
 * replaced by '_'), e.g. Examples/Model.Simulates/bricard_bricard_json.
 * test/meson.build registers one meson test per model so a crash in one model
 * does not hide the others.
 */
#include "gtest/gtest.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <string>
#include <vector>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"


namespace fs = std::filesystem;

static std::vector<std::string> FindExampleModels() {
  std::vector<std::string> models;
  const fs::path root(SHIFT_EXAMPLES_DIR);
  for(const auto& entry : fs::recursive_directory_iterator(root)) {
    if(entry.is_regular_file() && entry.path().extension() == ".json") {
      models.push_back(fs::relative(entry.path(), root).generic_string());
    }
  }
  std::sort(models.begin(), models.end());
  return models;
}

static std::string TestName(const testing::TestParamInfo<std::string>& info) {
  std::string name = info.param;
  for(char& c : name) {
    if(!std::isalnum(static_cast<unsigned char>(c))) {
      c = '_';
    }
  }
  return name;
}

class Model : public testing::TestWithParam<std::string> {
protected:
  void SetUp() override {
    // run each model in its own directory under the build tree so the log
    // files of different models (and parallel test runs) do not collide
    previous_dir_ = fs::current_path();
    output_dir_ = fs::path(SHIFT_BINARY_DIR) / "test" / "examples" /
                  TestName({GetParam(), 0});
    fs::create_directories(output_dir_);
    fs::current_path(output_dir_);
  }
  void TearDown() override {
    fs::current_path(previous_dir_);
  }

  fs::path previous_dir_;
  fs::path output_dir_;
};


TEST_P(Model, Simulates) {
  const std::string json_file =
    (fs::path(SHIFT_EXAMPLES_DIR) / GetParam()).string();
  try {
    shift::core::Context context;
    context.LoadJSON(json_file);
    context.Simulate();
  } catch(const shift::core::Exception& e) {
    FAIL() << json_file << ": " << e.what() << "\n" << e.backtrace();
  } catch(const std::exception& e) {
    FAIL() << json_file << ": " << e.what();
  }
}

INSTANTIATE_TEST_SUITE_P(Examples, Model, testing::ValuesIn(FindExampleModels()),
                         TestName);
