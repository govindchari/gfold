#pragma once
#include "gnc/FixedTOF.hpp"

/**
 * @brief Computes the free time of flight optimal trajectory. Currently just
 * takes the minimum time of flight feasible solution.
 *
 * @param r0 Initial position
 * @param v0 Initial velocity
 * @param m0 Initial mass
 * @param tof_guess A guess for the optimal time-of-flight (previous optimal tof minus dt)
 * @param data Struct containing trajectory, control inputs, and terminal mass
 * @return First thrust command
 */
Vector3d FreeTOF(const Vector3d &r0, const Vector3d &v0, const double &m0,
                double &tof_guess, FixedTOFData &data);