#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>

#include "test/contact/contact_record.h"
#include "gtest/gtest.h"

int main(int argc, char** argv) {
  /* a temporary directory of this process's own (testing::TempDir), so
  that shards running in parallel do not write the same files */
  std::error_code error;
  const char* tmpdir = std::getenv("TMPDIR");
  const std::filesystem::path own =
    std::filesystem::path(tmpdir && *tmpdir ? tmpdir : "/tmp") /
    ("shift-test-" + std::to_string(getpid()));
  if(std::filesystem::create_directories(own, error) || !error) {
    setenv("TEST_TMPDIR", own.c_str(), 1);
  }
  ::testing::InitGoogleTest(&argc, argv);
  shift::test::InstallContactRecorder();
  int ret = RUN_ALL_TESTS();
  std::filesystem::remove_all(own, error);
  return ret;
}
