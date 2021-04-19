#pragma once
#include "Eigen/Dense"
using namespace Eigen;
/**
 * @brief Computes the desired moment on the vehicle and updates q_int
 *
 * @param q Vehicle orientation quaternion
 * @param w Vehicle Angular Velocity
 * @param q_int Integral of the vector portion of the orientation quaternion
 * @return Desired moment on the vehicle
 */
Vector3d AttitudeController(const Vector4d &q, const Vector3d &w,
                            Vector3d &q_int);
