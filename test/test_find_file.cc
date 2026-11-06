#include "shift/core/utility.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

/* a tree with the same file name in the model's include directory and in the
directory of the same name under the working directory, so a lookup from the
working directory gives the wrong file:

  root/models/model.json
  root/models/assets/part.obj
  root/models/next_to.obj
  root/work/assets/part.obj   (decoy)
  root/shared/other.obj

The working directory is root/work for the length of a test. */
class FindFileTest : public testing::Test {
protected:
  void SetUp() override {
    start_ = std::filesystem::current_path();
    root_ = std::filesystem::path(testing::TempDir()) / "shift_find_file";
    std::filesystem::remove_all(root_);
    for(const char* directory :
        {"models/assets", "work/assets", "shared"}) {
      std::filesystem::create_directories(root_ / directory);
    }
    for(const char* file :
        {"models/model.json", "models/assets/part.obj", "models/next_to.obj",
         "work/assets/part.obj", "shared/other.obj"}) {
      std::ofstream(root_ / file) << "x";
    }
    std::filesystem::current_path(root_ / "work");
    model_ = (root_ / "models/model.json").string();
  }

  void TearDown() override {
    std::filesystem::current_path(start_);
    std::filesystem::remove_all(root_);
  }

  static std::filesystem::path Canonical(const std::string& file) {
    return std::filesystem::canonical(file);
  }

  std::filesystem::path start_;
  std::filesystem::path root_;
  std::string model_;
};

TEST(ParentPath, BareFileNameIsTheCurrentDirectory) {
  EXPECT_EQ(shift::core::utility::GetParentPath("model.json"), ".");
  EXPECT_EQ(shift::core::utility::GetParentPath(""), ".");
  EXPECT_EQ(shift::core::utility::GetParentPath("a/b/model.json"), "a/b");
  EXPECT_EQ(shift::core::utility::GetParentPath("/model.json"), "");
}

TEST_F(FindFileTest, NextToTheModelFile) {
  const std::string file =
    shift::core::utility::FindFile("next_to.obj", model_, {});
  ASSERT_FALSE(file.empty());
  EXPECT_EQ(Canonical(file), Canonical((root_ / "models/next_to.obj").string()));
}

TEST_F(FindFileTest, RelativeIncludeIsTakenFromTheModelFile) {
  const std::string file =
    shift::core::utility::FindFile("part.obj", model_, {"assets"});
  ASSERT_FALSE(file.empty());
  EXPECT_EQ(Canonical(file),
            Canonical((root_ / "models/assets/part.obj").string()));
}

TEST_F(FindFileTest, RelativeIncludeMayLeaveTheModelDirectory) {
  const std::string file =
    shift::core::utility::FindFile("other.obj", model_, {"../shared"});
  ASSERT_FALSE(file.empty());
  EXPECT_EQ(Canonical(file), Canonical((root_ / "shared/other.obj").string()));
}

TEST_F(FindFileTest, WorkingDirectoryIsNotSearched) {
  EXPECT_TRUE(shift::core::utility::FindFile("assets/part.obj",
                                             (root_ / "shared/m.json").string(),
                                             {"."})
                .empty());
}

TEST_F(FindFileTest, AbsoluteIncludeStandsAsItIs) {
  const std::string file = shift::core::utility::FindFile(
    "other.obj", model_, {(root_ / "shared").string()});
  ASSERT_FALSE(file.empty());
  EXPECT_EQ(Canonical(file), Canonical((root_ / "shared/other.obj").string()));
}

TEST_F(FindFileTest, AbsoluteNameStandsAsItIs) {
  const std::string name = (root_ / "shared/other.obj").string();
  EXPECT_EQ(shift::core::utility::FindFile(name, model_, {}), name);
  EXPECT_TRUE(shift::core::utility::FindFile(
                (root_ / "shared/missing.obj").string(), model_, {})
                .empty());
}

TEST_F(FindFileTest, BareModelFileName) {
  std::filesystem::current_path(root_ / "models");
  const std::string file =
    shift::core::utility::FindFile("part.obj", "model.json", {"assets"});
  ASSERT_FALSE(file.empty());
  EXPECT_EQ(Canonical(file),
            Canonical((root_ / "models/assets/part.obj").string()));
}

TEST_F(FindFileTest, MissingFile) {
  EXPECT_TRUE(
    shift::core::utility::FindFile("missing.obj", model_, {".", "assets"})
      .empty());
}
