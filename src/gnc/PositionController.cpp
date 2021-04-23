#include "gnc/PositionController.hpp"
#include "gnc/constants.hpp"

Vector3d PositionController(const Vector3d &pos, const Vector3d &vel,
                            Vector3d &pos_int) {
  Vector3d T_des;
  Vector3d weight;
  weight << 0,0,(vprop::m_dry+vprop::m_fuel)*environment::g;
  T_des = -Position::K_p * pos - Position::K_i * pos_int - Position::K_d * vel;
  T_des += weight;

  if (T_des(2,0)<0){
    T_des(2,0)=0;
  }
  // Update pos_int
  pos_int += pos * FlightComputer::dt;
  return T_des;
}