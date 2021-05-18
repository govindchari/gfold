#pragma once
#include "Eigen/Dense"
#include "gnc/utilities.hpp"
#include "nsim/constants.hpp"

/**
 * @brief Adds gaussian noise to the net force and moment vector
 * 
 * @param F Net Force
 * @param M Net Moment
 */
void ProcessNoise(Vector3d &F, Vector3d &M);