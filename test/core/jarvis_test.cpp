//          Copyright Diego Ramírez October 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/jarvis.hpp>
#include <gtest/gtest.h>

using namespace jarvis;

TEST(SumTest, WorksWell) {
  EXPECT_EQ(12, sum(5, 7));
  EXPECT_EQ(19, sum(10, 9));
  EXPECT_EQ(25, sum(12, 13));
  EXPECT_EQ(0, sum(0, 0));
  EXPECT_EQ(13, sum(13, 0));
  EXPECT_EQ(100, sum(50, 50));
  EXPECT_EQ(100, sum(90, 10));
}
