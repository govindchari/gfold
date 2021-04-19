#pragma once
#include "nsim/constants.hpp"
#include "gnc/utilities.hpp"

using namespace Eigen;

/**
 * @brief Applied Newton's Laws and Euler's Equations to generate the state
 * derivative
 *
 * @param state State Vector
 * @param F Net Force
 * @param M Net Moment
 * @return Derivative of State Vector
 */
Matrix<double, sim::num_states_6DOF, 1> eom(const Matrix<double, sim::num_states_6DOF, 1> state, const Vector3d F,
                          const Vector3d M) {
  // Extracting Vectors from State
  Vector3d r = state.block<3, 1>(0, 0);
  Vector4d q = state.block<4, 1>(3, 0);
  Vector3d v = state.block<3, 1>(7, 0);
  Vector3d w = state.block<3, 1>(10, 0);

  // Constructing quaternion representation of w
  Vector4d w_q;
  w_q << 0, w;

  Vector4d qd = 0.5 * hamilton(q, w_q);
  Vector3d vd = F / vprop::m_dry;
  Vector3d wd;
  wd << (M(0, 0) + (vprop::I2 - vprop::I3) * w(1, 0) * w(2, 0)) / vprop::I1,
      (M(1, 0) + (vprop::I3 - vprop::I1) * w(2, 0) * w(0, 0)) / vprop::I2,
      (M(2, 0) + (vprop::I1 - vprop::I2) * w(0, 0) * w(1, 0)) / vprop::I3;


  Matrix<double, sim::num_states_6DOF, 1> state_d;

  state_d << v, qd, vd, wd;

  return state_d;
}
