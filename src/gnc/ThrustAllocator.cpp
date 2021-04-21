#include "gnc/ThrustAllocator.hpp"
#include <iostream>

void ThrustAllocator(Vector3d &F_des, Vector3d &M_des, EngineActuation &actuation) {
  Vector3d T_b; // Thrust in the body frame
  Vector3d T_g; // Thrust in the gimbal frame
  Vector3d nhat;
  double theta;

  T_g << 0, 0, F_des.norm();
  double T_x = M_des(1, 0) / vprop::l;
  double T_y = -M_des(0, 0) / vprop::l;
  double T_z = sqrt(pow(T_x,2) + pow(T_y,2));
  T_b << T_x, T_y, T_z;
  std::cout << "T_body:" << std::endl;
  std::cout << T_b << std::endl;
  std::cout << "" << std::endl;

  nhat = cross(T_g, T_b);
  nhat.normalize();
  theta = asin(dot(T_g, T_b) / pow(F_des.norm(), 2));

  actuation.throttle = F_des.norm();
  actuation.axis = nhat;
  actuation.angle = theta;
}