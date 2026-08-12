#include <cmath>

#include "gtest/gtest.h"
#include "robot_navigation/intercept_math.hpp"

TEST(InterceptMath, SolvesStationaryTarget)
{
  const auto time = robot_navigation::solveInterceptTime(3.0, 0.0, 0.0, 0.0, 1.0);
  ASSERT_TRUE(time.has_value());
  EXPECT_NEAR(*time, 3.0, 1.0e-9);
}

TEST(InterceptMath, LeadsRecedingTarget)
{
  const auto time = robot_navigation::solveInterceptTime(3.0, 0.0, 0.5, 0.0, 1.0);
  ASSERT_TRUE(time.has_value());
  EXPECT_NEAR(*time, 6.0, 1.0e-9);
}

TEST(InterceptMath, RejectsUnreachableTarget)
{
  EXPECT_FALSE(
    robot_navigation::solveInterceptTime(3.0, 0.0, 1.5, 0.0, 1.0).has_value());
}

TEST(InterceptMath, CommandsStraightMotionForCenteredGoal)
{
  const auto command = robot_navigation::pursuitCommand(
    3.0, 0.0, 0.0, 3.0, 1.0, 0.15, 0.75, 0.24, 0.6);
  EXPECT_NEAR(command.linear_velocity, 1.0, 1.0e-9);
  EXPECT_NEAR(command.angular_velocity, 0.0, 1.0e-9);
}

TEST(InterceptMath, RespectsAckermannCurvatureLimit)
{
  const auto command = robot_navigation::pursuitCommand(
    0.0, 0.1, 0.0, 1.0, 1.0, 0.15, 0.75, 0.24, 0.6);
  const double maximum_angular_velocity =
    command.linear_velocity * std::tan(0.6) / 0.24;
  EXPECT_LE(std::abs(command.angular_velocity), maximum_angular_velocity);
}
