#include "nsim/simulations/run6DOF.hpp"

void run6DOF(Matrix<double, sim::num_states_6DOF, 1> z) {

  EngineActuation actuation;
  ForcesMoments forces_moments;
  Vector4d q;
  Vector3d T_des;
  Vector3d M_des;
  Vector3d q_int;
  Vector3d pos_int; //Remove Later

  std::vector<Matrix<double, sim::num_states_6DOF, 1>> data;

  while (z(2,0) >= 0) {
    q = z.block<4,1>(1,0);
    T_des = PositionController(z.block<3, 1>(0, 0), z.block<3, 1>(7, 0), pos_int);
    M_des = AttitudeController(T_des, z.block<4, 1>(3, 0), z.block<3, 1>(10, 0), q_int);
    ThrustAllocator(T_des, M_des, actuation);
    GenerateForcesMoments(actuation, q, forces_moments);
    z = step(&eom6DOF, z, T_des, M_des, actuation.throttle);

    // Stores state to data vector
    data.push_back(z);
  }
  SaveData(data);
}