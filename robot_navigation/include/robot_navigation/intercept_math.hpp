#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace robot_navigation
{

struct ControlCommand
{
  double linear_velocity;
  double angular_velocity;
};

inline double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline std::optional<double> solveInterceptTime(
  const double relative_x,
  const double relative_y,
  const double target_velocity_x,
  const double target_velocity_y,
  const double interceptor_speed)
{
  if (interceptor_speed <= 0.0) {
    return std::nullopt;
  }

  const double a =
    target_velocity_x * target_velocity_x +
    target_velocity_y * target_velocity_y -
    interceptor_speed * interceptor_speed;
  const double b = 2.0 *
    (relative_x * target_velocity_x + relative_y * target_velocity_y);
  const double c = relative_x * relative_x + relative_y * relative_y;

  if (std::abs(a) < 1.0e-9) {
    if (std::abs(b) < 1.0e-9) {
      return c < 1.0e-9 ? std::optional<double>(0.0) : std::nullopt;
    }
    const double time = -c / b;
    return time >= 0.0 ? std::optional<double>(time) : std::nullopt;
  }

  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0) {
    return std::nullopt;
  }

  const double root = std::sqrt(discriminant);
  const double first = (-b - root) / (2.0 * a);
  const double second = (-b + root) / (2.0 * a);
  double result = std::numeric_limits<double>::infinity();
  if (first >= 0.0) {
    result = first;
  }
  if (second >= 0.0) {
    result = std::min(result, second);
  }
  return std::isfinite(result) ? std::optional<double>(result) : std::nullopt;
}

inline ControlCommand pursuitCommand(
  const double goal_x,
  const double goal_y,
  const double robot_yaw,
  const double target_distance,
  const double maximum_speed,
  const double minimum_speed,
  const double slowdown_distance,
  const double wheelbase,
  const double maximum_steering_angle)
{
  const double goal_distance = std::hypot(goal_x, goal_y);
  if (goal_distance < 1.0e-9 || maximum_speed <= 0.0) {
    return {0.0, 0.0};
  }

  const double bearing = std::atan2(goal_y, goal_x);
  const double heading_error = normalizeAngle(bearing - robot_yaw);
  const double distance_scale = std::clamp(
    target_distance / slowdown_distance,
    minimum_speed / maximum_speed,
    1.0);
  const double heading_scale = std::max(0.2, std::cos(heading_error));
  const double linear_velocity = maximum_speed * distance_scale * heading_scale;

  const double requested_curvature =
    2.0 * std::sin(heading_error) / goal_distance;
  const double maximum_curvature =
    std::tan(maximum_steering_angle) / wheelbase;
  const double curvature = std::clamp(
    requested_curvature, -maximum_curvature, maximum_curvature);

  return {linear_velocity, linear_velocity * curvature};
}

}  // namespace robot_navigation
