#include "gnc/PositionController.hpp"
#include "gnc/constants.hpp"

Vector3d PositionController(const Vector3d &pos, const Vector3d &vel,
                            Vector3d &pos_int) {
  Vector3d T_des;
  Vector3d weight;
  weight << 0,0,(massprop::m_dry+massprop::m_fuel)*environment::g;
  T_des = -position::K_p * pos - position::K_i * pos_int - position::K_d * vel;
  T_des += weight;

  if (T_des(2,0)<0){
    T_des(2,0)=0;
  }
  // Update pos_int
  pos_int += pos * FlightComputer::dt;
  return T_des;
}