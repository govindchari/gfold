#include "gnc/AttitudeController.hpp"
#include "gnc/constants.hpp"

Vector3d AttitudeController(const Vector3d &T_des, const Vector4d &q, const Vector3d &w,
                                   Vector3d &q_int, Vector4d &q_des) {
  Vector3d M_des;
  Vector3d v_des;
  Vector3d T_body;
  Vector3d nhat;
  double theta;
  
  // Vector part of orientation quaternion
  Vector3d v = q.block<3, 1>(1, 0);

  // Constucting desired orientation quaternion
  T_body << 0, 0, T_des.norm();
  theta = acos(dot(T_body, T_des) / pow(T_des.norm(), 2));
  nhat = cross(T_body, T_des);
  nhat.normalize();
  q_des << cos(theta/2), nhat(0,0)*sin(theta/2), nhat(1,0)*sin(theta/2), nhat(2,0)*sin(theta/2);
  v_des = q_des.block<3,1>(1,0);

  // The signs are pluses because the orientation quaternion indicates the direction to 
  //rotate to go from body to inertial
  M_des =  attitude::K_p * (v-v_des) + attitude::K_i * q_int - attitude::K_d * w;

  // Update q_int
  q_int += (v-v_des) * FlightComputer::dt;
  return M_des;
}
