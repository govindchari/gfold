#include "nsim/constants.hpp"
#include "nsim/eom.hpp"
#include "nsim/step.hpp"
#include "gnc/AttitudeController.hpp"
#include "gnc/PositionController.hpp"
#include "gnc/utilities.hpp"
#include <iostream>

int main() {
  Eigen::Matrix<double, 13, 1> z;
  Eigen::Vector3d F;
  Eigen::Vector3d M;
  Eigen::Vector3d M_des;
  Eigen::Vector3d F_des;
  Eigen::Vector3d q_int;
  Eigen::Vector3d pos_int;


  z << 1, 2, 3, 1, 0, 0, 0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  F << 0, 0, 0;
  M << 0, 0, 0;
  q_int << 0,0,0;
  pos_int << 0,0,0;

  M_des = AttitudeController(z.block<4,1>(3,0), z.block<3,1>(10,0), q_int);
  F_des = PositionController(z.block<3,1>(0,0), z.block<3,1>(7,0), pos_int);
  z = step(&eom, z, sim::h, F, M);

  std::cout << F_des << std::endl;
}