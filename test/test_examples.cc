/*
 * Loads and simulates every model in examples/.
 *
 * The models are discovered at run time; each one is a separate test case
 * named after its path relative to examples/ (non-alphanumeric characters
 * replaced by '_'), e.g. Examples/Model.Simulates/bricard_bricard_json.
 * test/meson.build registers one meson test per model so a crash in one model
 * does not hide the others.
 *
 * The regular suite only checks that each model loads and runs: meson sets
 * SHIFT_TEST_TIME_SPAN, and a model then runs for that many seconds (at least
 * three of its output intervals) instead of its whole time span. The
 * benchmarks leave it unset and run the models in full.
 */
#include "gtest/gtest.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/integrator/base.h"


namespace fs = std::filesystem;

static std::vector<std::string> FindExampleModels() {
  std::vector<std::string> models;
  const fs::path root(SHIFT_EXAMPLES_DIR);
  for(const auto& entry : fs::recursive_directory_iterator(root)) {
    const std::string extension = entry.path().extension().string();
    if(entry.is_regular_file() &&
       (extension == ".json" || extension == ".yaml" || extension == ".yml")) {
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
    if(const char* cap = std::getenv("SHIFT_TEST_TIME_SPAN")) {
      auto integrator = context["model_parameters"]["integrators"][0]
                          .get_data<shift::core::integrator::Base>();
      const double span = std::max(std::atof(cap),
                                   3.0 * integrator->get_time_step());
      integrator->set_time_span(std::min(integrator->get_time_span(), span));
    }
    context.Simulate();
  } catch(const shift::core::Exception& e) {
    FAIL() << json_file << ": " << e.what() << "\n" << e.backtrace();
  } catch(const std::exception& e) {
    FAIL() << json_file << ": " << e.what();
  }
}

INSTANTIATE_TEST_SUITE_P(Examples, Model, testing::ValuesIn(FindExampleModels()),
                         TestName);
