#pragma once
#include "nsim/constants.hpp"
#include <Eigen/StdVector>
#include <fstream>

// Saves Data to .csv
void SaveData(const std::vector<Eigen::Matrix<double, 35, 1>> &data);