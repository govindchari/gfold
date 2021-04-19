#include "nsim/constants.hpp"
#include <Eigen/StdVector>
#include <fstream>

// Saves Data to .csv
void SaveData(const std::vector<Eigen::Matrix<double, sim::num_states_3DOF, 1>> &data);

void SaveData(const std::vector<Eigen::Matrix<double, sim::num_states_6DOF, 1>> &data);