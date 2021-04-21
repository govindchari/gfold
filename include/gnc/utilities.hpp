#pragma once
#include "Eigen/Dense"
using namespace Eigen;
/**
 * @brief Returns quaternion conjugate of input quaternion
 * 
 * @param q Input quaternion
 * @return Conjugated quaternion 
 */
static Vector4d conjugate(const Vector4d &q) {
    Vector4d result;
    result << q(0,0), -q(1,0), -q(2,0), -q(3,0);
    return result;
}

/**
 * @brief Performs hamilton product of two quaternions
 * 
 * @param q quaternion 1
 * @param r quaternion 2
 * @return Product of quaternion 1 and quaternion 2 
 */
static Vector4d hamilton(const Vector4d &q,const Vector4d &r) {
    Vector4d result;
    result <<r(0,0)*q(0,0)-r(1,0)*q(1,0)-r(2,0)*q(2,0)-r(3,0)*q(3,0),
             r(0,0)*q(1,0)+r(1,0)*q(0,0)-r(2,0)*q(3,0)+r(3,0)*q(2,0),
             r(0,0)*q(2,0)+r(1,0)*q(3,0)+r(2,0)*q(0,0)-r(3,0)*q(1,0),
             r(0,0)*q(3,0)-r(1,0)*q(2,0)+r(2,0)*q(1,0)+r(3,0)*q(0,0);
    return result;
}

/**
 * @brief Performs a change of basis operation on vector v
 * 
 * @param q quaternion which maps from frame A to frame B (B_q_A)
 * @param v Input vector in frame A
 * @return Input vector in frame B 
 */
static Vector3d rotate_frame(const Vector4d &q, const Vector3d &v) {
    Vector4d result_q;
    Vector3d result;
    Vector4d v_q;
    v_q << 0, v;
    result_q = hamilton(conjugate(q), hamilton(v_q, q));
    result << result_q(1,0), result_q(2,0), result_q(3,0);
    return result;
}

static Vector3d cross(const Vector3d &u, const Vector3d &v) {
    Vector3d result;
    result << u(1,0)*v(2,0)-u(2,0)*v(1,0),
              u(2,0)*v(0,0)-u(0,0)*v(2,0),
              u(0,0)*v(1,0)-u(1,0)*v(0,0);
    return result;
}

static double dot(const Vector3d &u, const Vector3d &v) {
    return (u(0,0)*v(0,0)+u(1,0)*v(1,0)+u(2,0)*v(2,0));
}
