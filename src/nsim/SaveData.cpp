#include "nsim/SaveData.hpp"
#include "gnc/constants.hpp"
#include "nsim/constants.hpp"

void SaveData(const std::vector<Eigen::Matrix<double, state_size, 1>> &data) {
  std::ofstream output_file("./results/data.csv");
  int j = 1;
  std::for_each(data.begin(), data.end(), [&](auto const &x) {
    output_file << j * sim::h;
    output_file << ",";
    for (int i = 0; i < state_size; i++) {
      output_file << x(i, 0);
      output_file << ",";
      if (i == state_size - 1) {
        output_file << std::endl;
      }
    }
    j++;
  });
}