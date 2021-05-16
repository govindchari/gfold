#include "nsim/simulations/run6DOF.hpp"
#include <iostream>

void run6DOF(Matrix<double, sim::num_states_6DOF, 1> z) {

  Matrix<double, 15, 1> info;
  Matrix<double, sim::num_states_6DOF + 6, 1> state_forces_moments;

  EngineActuation actuation;
  ForcesMoments forces_moments;
  Vector4d q;
  Vector4d q_des;
  Vector3d T_des;
  Vector3d M_des;
  Vector3d q_int;
  FixedTOFData data;
  double tof_guess = 100000;

  Vector3d zeros;
  Vector3d grav;
  zeros<<0, 0, 0;
  grav << 0, 0, environment::g;


  std::vector<Matrix<double, 15, 1>> fc_data;
  std::vector<Matrix<double, sim::num_states_6DOF + 6, 1>> sim_data;


  while (z(2,0) >= position::tolerance) {
    q = z.block<4,1>(3,0);

    T_des = FreeTOF(z.block<3, 1>(0, 0), z.block<3, 1>(7, 0), z(13,0), tof_guess, data);
    M_des = AttitudeController(T_des, z.block<4, 1>(3, 0), z.block<3, 1>(10, 0), q_int, q_des);
    ThrustAllocator(T_des, M_des, actuation);
    for (int i = 1; i <= FlightComputer::dt / sim::h; i++) {
      GenerateForcesMoments(actuation, q, z(13,0), forces_moments);
      z = step(&eom6DOF, z, forces_moments.force, forces_moments.moment, actuation.throttle);
      state_forces_moments << z, forces_moments.force, forces_moments.moment;
      sim_data.push_back(state_forces_moments);
    }
    info << T_des, M_des, actuation.throttle, actuation.axis, actuation.angle, q_des;
    
    // Stores state to data vector
    fc_data.push_back(info);
  }
  SaveData(fc_data);
  SaveData(sim_data);
}