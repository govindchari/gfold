#pragma once
#include "Eigen/Dense"
/**
 * @brief Computes the desired moment on the vehicle
 *
 * @param q Orientation quaternion
 * @param w Vehicle Angular Velocity
 * @param q_int Integral of the vector portion of the orientation quaternion
 * @return Desired moment on the vehicle
 */
Eigen::Vector3d AttitudeController(Eigen::Vector4d q, Eigen::Vector3d w,
                                   Eigen::Vector3d &q_int);
