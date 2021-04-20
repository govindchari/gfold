#include "nsim/simulations/run6DOF.hpp"

void run6DOF(Matrix<double, sim::num_states_6DOF, 1> z) {
  Vector3d F_des;
  Vector3d M_des;
  Vector3d q_int;
  Vector3d pos_int; //Remove Later
  double Th;

  std::vector<Matrix<double, sim::num_states_6DOF, 1>> data;

  while (z(2,0) >= 0) {
    M_des = AttitudeController(z.block<4, 1>(3, 0), z.block<3, 1>(10, 0), q_int);
    F_des = PositionController(z.block<3, 1>(0, 0), z.block<3, 1>(7, 0), pos_int);
    Th = F_des.norm();
    z = step(&eom6DOF, z, F_des, M_des, Th);

    // Stores state to data vector
    data.push_back(z);
  }
  SaveData(data);
}