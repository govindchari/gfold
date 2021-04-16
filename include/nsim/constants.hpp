#pragma once
#include "../../lib/eigen/Eigen/Dense"
namespace mprop {
static constexpr double mass = 1.0;
static constexpr double I1 = 0.1;
static constexpr double I2 = 0.1;
static constexpr double I3 = 0.1;
} // namespace mprop

namespace sim {
static constexpr double dt = 0.1;
} // namespace sim

namespace environment {
static constexpr double g = 9.81;
} // namespace environment