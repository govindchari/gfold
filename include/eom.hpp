#pragma once
#include "../include/constants.hpp"
#include "../include/utilities.hpp"

using namespace Eigen;
using namespace mprop;

// State vector is position, quaternion, velocity, and angular rate
Matrix<double, 13, 1> eom(const Matrix<double, 13, 1> state, const Vector3d F,
                          const Vector3d M) {
  // Extracting Vectors from State
  Vector3d r = state.block<3, 1>(0, 0);
  Vector4d q = state.block<4, 1>(3, 0);
  Vector3d v = state.block<3, 1>(7, 0);
  Vector3d w = state.block<3, 1>(10, 0);

  // Constructing quaternion representation of w
  Vector4d w_q;
  w_q << 0, w(0, 0), w(1, 0), w(2, 0);

  Vector4d qd = 0.5 * hamilton(q, w_q);
  Vector3d vd = F / mass;
  Vector3d wd;
  wd << (M(0, 0) + (I2 - I3) * w(1, 0) * w(2, 0)) / I1,
      (M(1, 0) + (I3 - I1) * w(2, 0) * w(0, 0)) / I2,
      (M(2, 0) + (I1 - I2) * w(0, 0) * w(1, 0)) / I3;

  Matrix<double, 13, 1> stated;
  stated << v, qd, vd, wd;

  return stated;
}
