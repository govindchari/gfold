#pragma once
#include "Eigen/Dense"
using namespace Eigen;

// Uses RK4 to step the simulation forward in time by h
template <typename T>
T step(T (*f)(T const &state, const Vector3d &force, const Vector3d &moment),
       const T &x, const double &h, const Vector3d &F, const Vector3d &M) {
  static constexpr double half = 0.5;
  static constexpr double two = 2.0;
  static constexpr double sixth = 1.0 / 6.0;
  auto k1 = f(x, F, M);
  auto k2 = f(x + half * h * k1, F, M);
  auto k3 = f(x + half * h * k2, F, M);
  auto k4 = f(x + h * k3, F, M);
  return x + (sixth)*h * (k1 + two * k2 + two * k3 + k4);
}