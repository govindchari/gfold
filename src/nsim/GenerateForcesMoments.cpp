#include "nsim/GenerateForcesMoments.hpp"
#include <iostream>

void GenerateForcesMoments(EngineActuation actuation, Vector4d &q, ForcesMoments &forces_moments) {
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
    b_q_g << cos(angle/2), -axis(0,0)*sin(angle/2), -axis(1,0)*sin(angle/2), -axis(2,0)*sin(angle/2);
    r << 0, 0, -vprop::l;
    grav << 0, 0, environment::g;
    T_body = rotate_frame(b_q_g, T_gimbal);
    T_inertial = rotate_frame(conjugate(q), T_body);

    // Computing net inertial force and net moment
    F_inertial = T_inertial - grav;
    M_body = cross(r, T_body);
    forces_moments.force = F_inertial;
    forces_moments.moment = M_body;

}