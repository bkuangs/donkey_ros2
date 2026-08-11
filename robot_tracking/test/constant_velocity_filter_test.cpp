#include "gtest/gtest.h"
#include "robot_tracking/constant_velocity_filter.hpp"

TEST(ConstantVelocityFilter, ConvergesOnConstantVelocity)
{
  robot_tracking::ConstantVelocityFilter filter;
  filter.initialize(0.0, 0.0, 0.01, 0.01);

  for (int step = 1; step <= 50; ++step) {
    const double time = 0.1 * step;
    ASSERT_TRUE(filter.update(
      0.4 * time, -0.2 * time, 0.01, 0.01, 0.1, 0.1, 25.0));
  }

  EXPECT_NEAR(filter.state()(2), 0.4, 0.03);
  EXPECT_NEAR(filter.state()(3), -0.2, 0.03);
}

TEST(ConstantVelocityFilter, RejectsLargeInnovation)
{
  robot_tracking::ConstantVelocityFilter filter;
  filter.initialize(0.0, 0.0, 0.01, 0.01);

  EXPECT_FALSE(filter.update(
    100.0, 100.0, 0.01, 0.01, 0.1, 0.1, 25.0));
}
