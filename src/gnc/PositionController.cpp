#include "gnc/PositionController.hpp"
#include "gnc/constants.hpp"

Vector3d PositionController(const Vector3d &pos, const Vector3d &vel,
                            Vector3d &pos_int) {
  Vector3d F_des;
  F_des = -Position::K_p * pos - Position::K_i * pos_int - Position::K_d * vel;

  // Update pos_int
  pos_int += pos * FlightComputer::dt;
  return F_des;
}