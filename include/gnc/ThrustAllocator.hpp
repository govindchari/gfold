#pragma once
#include "Eigen/Dense"
#include "gnc/constants.hpp"
#include "gnc/utilities.hpp"
using namespace Eigen;

struct EngineActuation {
  double throttle;
  Vector3d axis;
  double angle;
};

/**
 * @brief Computes the throttle and TVC angle from the desired thrust vector and
 *  desired moment. The actuation struct is mutated.
 *
 * @param T_des Desired thrust vector in inertial coordinates
 * @param M_des Desired moment on the vehicle
 * @param actuation Contains the thrust magnitude (throttle) and the axis and
 * angle that the gimbal will rotate with respect to the body frame
 */
void ThrustAllocator(Vector3d &T_des, Vector3d &M_des,
                            EngineActuation &actuation);