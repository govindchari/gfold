#include "gnc/AttitudeController.hpp"
#include "gnc/constants.hpp"

Eigen::Vector3d AttitudeController(const Eigen::Vector4d &q, const Eigen::Vector3d &w,
                                   Eigen::Vector3d &q_int) {
  Eigen::Vector3d M_des;
  Eigen::Vector3d v = q.block<3, 1>(1, 0);
  
  // The signs are pluses because the orientation quaternion indicates the direction to 
  //rotate to go from body to inertial
  M_des = Attitude::K_p * v + Attitude::K_i * q_int + Attitude::K_d * w;

  // Update q_int
  q_int += v * FlightComputer::dt;
  return M_des;
}
