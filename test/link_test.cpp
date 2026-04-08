#include "../src/kin/link.h"
#include <gtest/gtest.h>

TEST(LinkTest, testLinkConstruct)
{
  rb::kin::Link lk = rb::kin::Link(0.0, 180., -335.0, 0.);
  ASSERT_EQ(335.0 , lk.tf(2,3));
}

TEST(LinkTest, testLinkComputeTransform)
{
  rb::kin::Link lk_1 = rb::kin::Link(0.0, 180., -335.0, 0.);
  lk_1.computeTransform(90.0, true);
  rb::kin::Link lk_2 = rb::kin::Link(75., 90., 0., -90);
  auto lk_tf = lk_1 * lk_2;
  ASSERT_LE(-75 - lk_tf(1,3), rb::math::EPSILON);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
