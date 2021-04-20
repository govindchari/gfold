#include <chrono>
#include <iostream>
#include "nsim/simulations/run6DOF.hpp"


int main() {

  // Initial Condition
  Matrix<double, sim::num_states_6DOF, 1> z;
  z << 100, 200, 300, 1, 0, 0, 0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, vprop::m_dry + vprop ::m_fuel;
  auto start = std::chrono::high_resolution_clock::now();
  
  run6DOF(z);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << duration.count() << std::endl;

}