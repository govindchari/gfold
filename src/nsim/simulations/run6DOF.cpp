#include "nsim/simulations/run6DOF.hpp"

void run6DOF(Matrix<double, sim::num_states_6DOF, 1> z) {
  Vector3d T_des;
  Vector3d M_des;
  Vector3d q_int;
  Vector3d pos_int; //Remove Later
  double T_mag;

  std::vector<Matrix<double, sim::num_states_6DOF, 1>> data;

  while (z(2,0) >= 0) {
    T_des = PositionController(z.block<3, 1>(0, 0), z.block<3, 1>(7, 0), pos_int);
    M_des = AttitudeController(T_des, z.block<4, 1>(3, 0), z.block<3, 1>(10, 0), q_int);
    T_mag = T_des.norm();
    z = step(&eom6DOF, z, T_des, M_des, T_mag);

    // Stores state to data vector
    data.push_back(z);
  }
  SaveData(data);
}