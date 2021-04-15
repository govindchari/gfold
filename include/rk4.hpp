#include "../lib/eigen/Eigen/Dense"
using namespace Eigen;

template <typename T> T rk4(T (*f)(T state), T x, double h) {
  static constexpr double half = 0.5;
  static constexpr double two = 2.0;
  static constexpr double sixth = 1.0 / 6.0;
  auto k1 = f(x);
  auto k2 = f(x + half * h * k1);
  auto k3 = f(x + half * h * k2);
  auto k4 = f(x + h * k3);
  return (sixth) * h * (k1 + two * k2 + two * k3 + k4);
}