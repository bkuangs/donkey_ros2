#pragma once

#include "Eigen/Dense"

namespace robot_tracking
{

class ConstantVelocityFilter
{
public:
  using Vector4d = Eigen::Matrix<double, 4, 1>;
  using Matrix4d = Eigen::Matrix<double, 4, 4>;

  void initialize(
    const double x,
    const double y,
    const double x_variance,
    const double y_variance)
  {
    state_ << x, y, 0.0, 0.0;
    covariance_.setZero();
    covariance_(0, 0) = x_variance;
    covariance_(1, 1) = y_variance;
    covariance_(2, 2) = 4.0;
    covariance_(3, 3) = 4.0;
    initialized_ = true;
  }

  bool update(
    const double measured_x,
    const double measured_y,
    const double x_variance,
    const double y_variance,
    const double dt,
    const double acceleration_variance,
    const double innovation_gate)
  {
    const Matrix4d state_transition = transition(dt);
    state_ = state_transition * state_;
    covariance_ =
      state_transition * covariance_ * state_transition.transpose() +
      processNoise(dt, acceleration_variance);

    Eigen::Matrix<double, 2, 4> observation =
      Eigen::Matrix<double, 2, 4>::Zero();
    observation(0, 0) = 1.0;
    observation(1, 1) = 1.0;

    Eigen::Matrix2d measurement_noise = Eigen::Matrix2d::Zero();
    measurement_noise(0, 0) = x_variance;
    measurement_noise(1, 1) = y_variance;
    const Eigen::Vector2d innovation =
      Eigen::Vector2d(measured_x, measured_y) - observation * state_;
    const Eigen::Matrix2d innovation_covariance =
      observation * covariance_ * observation.transpose() + measurement_noise;
    const auto decomposition = innovation_covariance.ldlt();
    const double squared_mahalanobis =
      innovation.dot(decomposition.solve(innovation));
    if (squared_mahalanobis > innovation_gate) {
      return false;
    }

    const Eigen::Matrix<double, 4, 2> gain =
      covariance_ * observation.transpose() *
      decomposition.solve(Eigen::Matrix2d::Identity());
    state_ += gain * innovation;
    const Matrix4d identity = Matrix4d::Identity();
    covariance_ =
      (identity - gain * observation) * covariance_ *
      (identity - gain * observation).transpose() +
      gain * measurement_noise * gain.transpose();
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
    return true;
  }

  const Vector4d & state() const
  {
    return state_;
  }

  const Matrix4d & covariance() const
  {
    return covariance_;
  }

  bool initialized() const
  {
    return initialized_;
  }

private:
  static Matrix4d transition(const double dt)
  {
    Matrix4d result = Matrix4d::Identity();
    result(0, 2) = dt;
    result(1, 3) = dt;
    return result;
  }

  static Matrix4d processNoise(
    const double dt,
    const double acceleration_variance)
  {
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    Matrix4d noise = Matrix4d::Zero();
    noise(0, 0) = dt4 / 4.0;
    noise(0, 2) = dt3 / 2.0;
    noise(2, 0) = dt3 / 2.0;
    noise(2, 2) = dt2;
    noise(1, 1) = dt4 / 4.0;
    noise(1, 3) = dt3 / 2.0;
    noise(3, 1) = dt3 / 2.0;
    noise(3, 3) = dt2;
    return acceleration_variance * noise;
  }

  Vector4d state_ = Vector4d::Zero();
  Matrix4d covariance_ = Matrix4d::Identity();
  bool initialized_ = false;
};

}  // namespace robot_tracking
