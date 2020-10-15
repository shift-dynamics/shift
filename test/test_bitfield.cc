#include "gtest/gtest.h"
#include <iostream>

#include "shift/core/bitfield.h"

TEST(BitField, SetTrue) {
  shift::core::BitField b = shift::core::BitField(37);
  b.print();

  b.setTrue(3);
  b.setTrue(13);
  b.setTrue(21);

  b.print();

  EXPECT_TRUE(b.get(3));
  EXPECT_TRUE(b.get(13));
  EXPECT_TRUE(b.get(21));

  EXPECT_FALSE(b.get(4));
  EXPECT_FALSE(b.get(14));
  EXPECT_FALSE(b.get(22));

  b.setFalse(3);
  EXPECT_FALSE(b.get(3));

  b.clear();
  EXPECT_FALSE(b.get(13));
  EXPECT_FALSE(b.get(21));
}
