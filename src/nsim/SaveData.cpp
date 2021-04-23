#include "nsim/SaveData.hpp"
#include "nsim/constants.hpp"

void SaveData(const std::vector<Eigen::Matrix<double, sim::num_states_6DOF + 21, 1>> &data) {
  std::ofstream output_file("./results/data.csv");
  int j = 1;
  std::for_each(data.begin(), data.end(), [&](auto const &x) {
    output_file << j * sim::h;
    output_file << ",";
    for (int i = 0; i < sim::num_states_6DOF + 21; i++) {
      output_file << x(i, 0);
      output_file << ",";
      if (i == sim::num_states_6DOF + 21 - 1) {
        output_file << std::endl;
      }
    }
    j++;
  });
}
