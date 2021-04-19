#pragma once
#include "Eigen/Dense"
using namespace Eigen;

// Uses RK4 to step the simulation forward in time by sim::h
template <typename T>
T step(T (*f)(T const &state, const Vector3d &force, const Vector3d &moment, const double &thrust),
       const T &x, const Vector3d &F, const Vector3d &M, const double &Th) {
  static constexpr double half = 0.5;
  static constexpr double two = 2.0;
  static constexpr double sixth = 1.0 / 6.0;
  auto k1 = f(x, F, M, Th);
  auto k2 = f(x + half * sim::h * k1, F, M, Th);
  auto k3 = f(x + half * sim::h * k2, F, M, Th);
  auto k4 = f(x + sim::h * k3, F, M, Th);
  return x + (sixth)* sim::h * (k1 + two * k2 + two * k3 + k4);
}