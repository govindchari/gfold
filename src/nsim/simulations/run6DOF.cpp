#include "nsim/simulations/run6DOF.hpp"

void run6DOF(Matrix<double, sim::num_states_6DOF, 1> z) {

  Matrix<double, sim::num_states_6DOF + 21, 1> info;
  EngineActuation actuation;
  ForcesMoments forces_moments;
  Vector4d q;
  Vector4d q_des;
  Vector3d T_des;
  Vector3d M_des;
  Vector3d q_int;
  Vector3d pos_int; //Remove Later

  std::vector<Matrix<double, sim::num_states_6DOF + 21, 1>> data;

  while (z(2,0) >= 0) {
    q = z.block<4,1>(3,0);

    T_des = PositionController(z.block<3, 1>(0, 0), z.block<3, 1>(7, 0), pos_int);
    M_des = AttitudeController(T_des, z.block<4, 1>(3, 0), z.block<3, 1>(10, 0), q_int, q_des);
    ThrustAllocator(T_des, M_des, actuation);
    GenerateForcesMoments(actuation, q, z(13,0), forces_moments);
    z = step(&eom6DOF, z, forces_moments.force, forces_moments.moment, actuation.throttle);
    info << z, T_des, M_des, actuation.throttle, actuation.axis, actuation.angle, forces_moments.force, forces_moments.moment, q_des;

    // Stores state to data vector
    data.push_back(info);
  }
  SaveData(data);
}