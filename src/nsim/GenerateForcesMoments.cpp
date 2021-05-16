#include "nsim/GenerateForcesMoments.hpp"
#include <iostream>

void GenerateForcesMoments(EngineActuation actuation, const Vector4d &q, const double &m, ForcesMoments &forces_moments) {
    // Pulling data from struct
    double throttle = actuation.throttle;
    Vector3d axis = actuation.axis;
    double angle = actuation.angle;

    // Declaring necessary variables
    Vector3d T_gimbal;
    Vector3d T_body;
    Vector3d T_inertial;
    Vector3d F_inertial;
    Vector3d M_body;
    Vector3d grav;
    Vector4d b_q_g;
    Vector3d r;

    // Defining necessary variables
    T_gimbal << 0, 0, throttle;

    // Dealing with the case if there is no gimbal actuation
    if (actuation.axis.norm() <= 1e-5) {
        T_body = T_gimbal;
    }else {
        b_q_g << cos(angle/2), -axis(0,0)*sin(angle/2), -axis(1,0)*sin(angle/2), -axis(2,0)*sin(angle/2);
        T_body = rotate_frame(b_q_g, T_gimbal);
    }
    r << 0, 0, -massprop::l;
    grav << 0, 0, environment::g;
    T_inertial = rotate_frame((q), T_body);

    // Computing net inertial force and net moment
    F_inertial = T_inertial - m * grav;
    M_body = cross(r, T_body);
    forces_moments.force = F_inertial;
    forces_moments.moment = M_body;

}