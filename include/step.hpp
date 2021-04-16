#pragma once
#include "../lib/eigen/Eigen/Dense"
using namespace Eigen;

// Uses RK4 to step the simulation forward in time by h
template <typename T>
T step(T (*f)(T state, Vector3d force, Vector3d moment), T x, double h,
       Vector3d F, Vector3d M) {
  static constexpr double half = 0.5;
  static constexpr double two = 2.0;
  static constexpr double sixth = 1.0 / 6.0;
  auto k1 = f(x, F, M);
  auto k2 = f(x + half * h * k1, F, M);
  auto k3 = f(x + half * h * k2, F, M);
  auto k4 = f(x + h * k3, F, M);
  return x + (sixth)*h * (k1 + two * k2 + two * k3 + k4);
}