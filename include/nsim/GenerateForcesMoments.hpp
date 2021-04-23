#pragma once
#include "Eigen/Dense"
#include "gnc/ThrustAllocator.hpp"
#include "gnc/utilities.hpp"

struct ForcesMoments {
    Vector3d force;
    Vector3d moment;
};

/**
 * @brief Computes the net inertial force on the vehicle and net moment (in body coordinates)
 * and mutates the forces_moments struct
 * 
 * @param actuation Struct containing throttle, TVC actuation axis, TVC actuation angle
 * @param q Orientation quaternion
 * @param forces_moments This field is mutated to contain the net force and moment
 */
void GenerateForcesMoments(EngineActuation actuation, const Vector4d &q, const double &m, ForcesMoments &forces_moments);