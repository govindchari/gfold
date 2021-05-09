#pragma once
#include "Eigen/Dense"
#include "epigraph.hpp"
#include "gnc/constants.hpp"

using namespace cvx;
using namespace Eigen;

/**
 * @brief Struct containing trajectory, control inputs, and terminal mass
 * 
 */
struct FixedTOFData {
  MatrixXd x_sol;
  MatrixXd u_sol;
  VectorXd z_sol;
  double m_final;
};

/**
 * @brief Uses convex optimization to solve for the fuel optimal trajectory given
 * initial position, velocity, mass, and time of flight
 * 
 * @param r0 Initial position
 * @param v0 Initial velocity
 * @param tof Time of flight
 * @param m0 Initial mass
 * @param data Struct containing optimal trajectory, control inputs, and final mass
 */
void FixedTOF(const Vector3d &r0, const Vector3d &v0, const double &tof,
              const double &m0, FixedTOFData &data);
