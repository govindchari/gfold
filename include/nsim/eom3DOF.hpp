#pragma once
#include "gnc/utilities.hpp"
#include "nsim/constants.hpp"

using namespace Eigen;

/**
 * @brief Computes the state's derivative for a 3DOF simulation
 *
 * @param state position, velocity, mass
 * @param F Net Force on Vehicle
 * @param T Magnitude of Thrust Vector
 * @return Derivative of state
 */
Matrix<double, sim::num_states_3DOF, 1>
eom_3DOF(const Matrix<double, sim::num_states_3DOF, 1> &state,
         const Vector3d &F, const Vector3d &_, const double &T) {
  // Extracting Vectors from State
  Vector3d r = state.block<3, 1>(0, 0);
  Vector3d v = state.block<3, 1>(3, 0);
  double m = state(7, 0);

  // Computing derivatives
  Vector3d vd = F / m;
  double md = -T / (vprop::Isp * environment::g0);

  // Assembling state derivative
  Matrix<double, sim::num_states_6DOF, 1> state_d;
  state_d << v, vd, md;
  return state_d;
}
