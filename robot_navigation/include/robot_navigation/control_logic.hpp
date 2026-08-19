#pragma once

#include <cstdint>

namespace robot_navigation
{

enum class CommandOwner : uint8_t
{
  STOP = 0,
  NAV2 = 1,
  TERMINAL = 2,
};

inline bool inputIsFresh(
  const bool received, const double age, const double timeout)
{
  return received && age >= 0.0 && age <= timeout;
}

inline bool shouldForwardCommand(
  const CommandOwner owner,
  const CommandOwner source,
  const bool received,
  const double age,
  const double timeout)
{
  return owner == source && owner != CommandOwner::STOP &&
         inputIsFresh(received, age, timeout);
}

inline bool updateTerminalState(
  const bool terminal_active,
  const double distance,
  const double enter_distance,
  const double exit_distance)
{
  return terminal_active ? distance < exit_distance : distance <= enter_distance;
}

inline bool shouldRefreshGoal(
  const bool goal_active,
  const double elapsed,
  const double displacement,
  const double minimum_period,
  const double minimum_displacement)
{
  return elapsed >= minimum_period &&
         (!goal_active || displacement >= minimum_displacement);
}

}  // namespace robot_navigation
