#include "../lib/eigen/Eigen/Dense"
using namespace Eigen;

template <typename T>
T ode4(T (*function)(T state), T x, double h) {
    return x+h*function(x);
}