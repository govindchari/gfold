#include "nsim/simulations/run6DOF.hpp"
#include <iostream>

void run6DOF(Matrix<double, sim::num_states_6DOF, 1> z) {

  Matrix<double, sim::num_states_6DOF + 21, 1> info;
  EngineActuation actuation;
  ForcesMoments forces_moments;
  Vector4d q;
  Vector4d q_des;
  Vector3d T_des;
  Vector3d M_des;
  Vector3d q_int;
  FixedTOFData data;

  Vector3d zeros;
  Vector3d grav;
  zeros<<0,0,0;
  grav << 0,0,environment::g;


  std::vector<Matrix<double, sim::num_states_6DOF + 21, 1>> results;

  while (z(2,0) >= 0) {
    q = z.block<4,1>(3,0);

    /*
    std::cout << z.block<3, 1>(0, 0) << std::endl;
    std::cout << z.block<3, 1>(7, 0) << std::endl;
    std::cout << z(13,0) << std::endl;
    */

    T_des = FreeTOF(z.block<3, 1>(0, 0), z.block<3, 1>(7, 0), z(13,0), data);
    //M_des = AttitudeController(T_des, z.block<4, 1>(3, 0), z.block<3, 1>(10, 0), q_int, q_des);
    //ThrustAllocator(T_des, M_des, actuation);
    //GenerateForcesMoments(actuation, q, z(13,0), forces_moments);
    //z = step(&eom6DOF, z, forces_moments.force, forces_moments.moment, actuation.throttle);

    z = step(&eom6DOF, z, T_des - z(13,0)*grav, zeros, T_des.norm());
    //info << z, T_des, M_des, actuation.throttle, actuation.axis, actuation.angle, forces_moments.force, forces_moments.moment, q_des;
  
    // Stores state to data vector
    //results.push_back(info);
  }
  //SaveData(results);
}