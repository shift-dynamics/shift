#include "gtest/gtest.h"
#include <iostream>

#include "shift/core/bit_field.h"

TEST(BitField, SetTrue) {
  shift::core::BitField b = shift::core::BitField(37);
  b.Print();

  b.set_true(3);
  b.set_true(13);
  b.set_true(21);

  b.Print();

  EXPECT_TRUE(b.get(3));
  EXPECT_TRUE(b.get(13));
  EXPECT_TRUE(b.get(21));

  EXPECT_FALSE(b.get(4));
  EXPECT_FALSE(b.get(14));
  EXPECT_FALSE(b.get(22));

  b.set_false(3);
  EXPECT_FALSE(b.get(3));

  b.Clear();
  EXPECT_FALSE(b.get(13));
  EXPECT_FALSE(b.get(21));
}
