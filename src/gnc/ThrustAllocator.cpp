#include "gnc/ThrustAllocator.hpp"
#include <iostream>

void ThrustAllocator(Vector3d &T_des, Vector3d &M_des,
                            EngineActuation &actuation) {
  Vector3d T_body;   // Thrust in the body frame
  Vector3d T_gimbal; // Thrust in the gimbal frame
  Vector3d nhat;
  double theta;

  T_gimbal << 0, 0, T_des.norm();
  double T_x = - M_des(1, 0) / massprop::l;
  double T_y = M_des(0, 0) / massprop::l;
  double T_z = sqrt(pow(T_des.norm(), 2) - pow(T_x, 2) - pow(T_y, 2));
  T_body << T_x, T_y, T_z;
  
  // Dealing with numeric issue of an acos argument greater than unity
  if (dot(T_gimbal, T_body) / pow(T_des.norm(), 2) >= 1.0) {
    theta = 0;
  } else {
    theta = acos(dot(T_gimbal, T_body) / pow(T_des.norm(), 2));
  }

  if (theta == 0) {
    nhat << 0, 0, 0;
  } else {
    nhat = cross(T_gimbal, T_body);
    nhat.normalize();
  }

  actuation.throttle = T_des.norm();
  actuation.axis = nhat;
  actuation.angle = theta;

  // Saturation block
  if (actuation.throttle > propulsion::T_max) {
    actuation.throttle = propulsion::T_max;
  }
  if (actuation.throttle < propulsion::T_min) {
    actuation.throttle = propulsion::T_min;
  }
  if (actuation.angle > attitude::theta_max) {
    actuation.angle = attitude::theta_max;
  }
}