#include "gtest/gtest.h"
#include "robot_navigation/control_logic.hpp"

using robot_navigation::CommandOwner;

TEST(ControlLogic, AppliesTerminalRangeHysteresis)
{
  EXPECT_FALSE(robot_navigation::updateTerminalState(false, 1.01, 1.0, 1.25));
  EXPECT_TRUE(robot_navigation::updateTerminalState(false, 1.0, 1.0, 1.25));
  EXPECT_TRUE(robot_navigation::updateTerminalState(true, 1.24, 1.0, 1.25));
  EXPECT_FALSE(robot_navigation::updateTerminalState(true, 1.25, 1.0, 1.25));
}

TEST(ControlLogic, RejectsMissingStaleAndFutureInputs)
{
  EXPECT_FALSE(robot_navigation::inputIsFresh(false, 0.0, 0.25));
  EXPECT_TRUE(robot_navigation::inputIsFresh(true, 0.25, 0.25));
  EXPECT_FALSE(robot_navigation::inputIsFresh(true, 0.26, 0.25));
  EXPECT_FALSE(robot_navigation::inputIsFresh(true, -0.01, 0.25));
}

TEST(ControlLogic, ForwardsOnlyFreshOwnedCommands)
{
  EXPECT_TRUE(robot_navigation::ownerTransitionRequiresStop(
      CommandOwner::NAV2, CommandOwner::TERMINAL));
  EXPECT_FALSE(robot_navigation::ownerTransitionRequiresStop(
      CommandOwner::NAV2, CommandOwner::NAV2));
  EXPECT_TRUE(robot_navigation::shouldForwardCommand(
      CommandOwner::NAV2, CommandOwner::NAV2, true, 0.1, 0.25));
  EXPECT_FALSE(robot_navigation::shouldForwardCommand(
      CommandOwner::TERMINAL, CommandOwner::NAV2, true, 0.1, 0.25));
  EXPECT_FALSE(robot_navigation::shouldForwardCommand(
      CommandOwner::NAV2, CommandOwner::NAV2, true, 0.3, 0.25));
  EXPECT_FALSE(robot_navigation::shouldForwardCommand(
      CommandOwner::STOP, CommandOwner::STOP, true, 0.0, 0.25));
}

TEST(ControlLogic, RateLimitsGoalRefreshByTimeAndDisplacement)
{
  EXPECT_TRUE(robot_navigation::shouldRefreshGoal(
      false, false, 0.0, 0.0, 1.0, 0.5));
  EXPECT_FALSE(robot_navigation::shouldRefreshGoal(
      true, true, 0.9, 1.0, 1.0, 0.5));
  EXPECT_FALSE(robot_navigation::shouldRefreshGoal(
      true, true, 1.0, 0.49, 1.0, 0.5));
  EXPECT_TRUE(robot_navigation::shouldRefreshGoal(
      true, true, 1.0, 0.5, 1.0, 0.5));
  EXPECT_TRUE(robot_navigation::shouldRefreshGoal(
      true, false, 1.0, 0.0, 1.0, 0.5));
}
