#include "nsim/constants.hpp"
#include "nsim/eom.hpp"
#include "nsim/step.hpp"
#include "gnc/AttitudeController.hpp"
#include "gnc/utilities.hpp"
#include <iostream>

int main() {
  Eigen::Matrix<double, 13, 1> z;
  Eigen::Vector3d F;
  Eigen::Vector3d M;
  Eigen::Vector3d M_des;
  Eigen::Vector4d q;
  Eigen::Vector3d w;
  Eigen::Vector3d q_int;

  z << 1, 2, 3, 1, 0, 0, 0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  F << 0, 0, 0;
  M << 0, 0, 0;
  z = step(&eom, z, sim::h, F, M);
  q = z.block<4,1>(3,0);
  w = z.block<3,1>(7,0);
  q_int = z.block<3,1>(10,0);

  M_des = AttitudeController(q,w,q_int);
  std::cout << M_des << std::endl;
}