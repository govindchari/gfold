#pragma once
#include "Eigen/Dense"
using namespace Eigen;
/**
 * @brief Computes the desired thrust vector vehicle and updates pos_int
 *
 * @param pos Vehicle position
 * @param vel Vehicle velocity
 * @param pos_int Integrated position error
 * @return Desired thrust vector
 */
Vector3d PositionController(const Vector3d &pos, const Vector3d &vel,
                            Vector3d &pos_int);