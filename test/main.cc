#include "test/contact/contact_record.h"
#include "gtest/gtest.h"

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  shift::test::InstallContactRecorder();
  int ret = RUN_ALL_TESTS();
  return ret;
}
