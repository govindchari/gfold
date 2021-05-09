#pragma once
#include "Eigen/Dense"
#include "gnc/utilities.hpp"
using namespace Eigen;

/**
 * @brief Computes desired moment on vehicle, desired orientation quaternion, 
 * and updates integrated error
 * 
 * @param T_des Desired thrust vector
 * @param q Orientation quaternion
 * @param w Angular rate
 * @param q_int Integrated error
 * @param q_des Desired orientation quaternion
 * @return Vector3d 
 */
Vector3d AttitudeController(const Vector3d &T_des, const Vector4d &q, const Vector3d &w,
                            Vector3d &q_int, Vector4d &q_des);
