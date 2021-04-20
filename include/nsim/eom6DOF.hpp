#pragma once
#include "gnc/utilities.hpp"
#include "nsim/constants.hpp"

using namespace Eigen;

/**
 * @brief Computes the state's derivative for a 6DOF simulation
 * 
 * @param state position, velocity, orientation quaternion, angular rate, mass
 * @param F Net Force on Vehicle
 * @param M Net Moment on Vehicle
 * @param T Magnitude of Thrust Vector
 * @return Derivative of state
 */
static Matrix<double, sim::num_states_6DOF, 1>
eom6DOF(const Matrix<double, sim::num_states_6DOF, 1> &state, const Vector3d &F,
    const Vector3d &M, const double &T) {
  // Extracting Vectors from State
  Vector3d r = state.block<3, 1>(0, 0);
  Vector4d q = state.block<4, 1>(3, 0);
  Vector3d v = state.block<3, 1>(7, 0);
  Vector3d w = state.block<3, 1>(10, 0);
  double m = state(13,0);

  // Constructing quaternion representation of w
  Vector4d w_q;
  w_q << 0, w;

  // Computing derivatives
  Vector4d qd = 0.5 * hamilton(q, w_q);
  Vector3d vd = F / m;
  Vector3d wd;
  wd << (M(0, 0) + (vprop::I2 - vprop::I3) * w(1, 0) * w(2, 0)) / vprop::I1,
      (M(1, 0) + (vprop::I3 - vprop::I1) * w(2, 0) * w(0, 0)) / vprop::I2,
      (M(2, 0) + (vprop::I1 - vprop::I2) * w(0, 0) * w(1, 0)) / vprop::I3;
  double md = -T / (vprop::Isp * environment::g0);

  // Assembling state derivative
  Matrix<double, sim::num_states_6DOF, 1> state_d;
  state_d << v, qd, vd, wd, md;
  return state_d;
}
