#pragma once
#include "Eigen/Dense"
#include "gnc/utilities.hpp"
using namespace Eigen;

/**
 * @brief Computes the desired moment on the vehicle in order to have the vehicle
 * align with the desired thrust vector
 * 
 * @param T_des Desired thrust vector in an inertial frame
 * @param q Orientation quaternion
 * @param w Vehicle's angualar rate
 * @param q_int Integrated error of quaternion
 * @return Vector3d 
 */
Vector3d AttitudeController(const Vector3d &T_des, const Vector4d &q, const Vector3d &w,
                            Vector3d &q_int);
