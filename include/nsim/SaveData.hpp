#pragma once
#include "nsim/constants.hpp"
#include <Eigen/StdVector>
#include <fstream>

// Saves Data to .csv
void SaveData(const std::vector<Eigen::Matrix<double, 15, 1>> &data);
void SaveData(const std::vector<Eigen::Matrix<double, 20, 1>> &data);