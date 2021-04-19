#pragma once
namespace vprop {
static constexpr double m_dry = 1.0;
static constexpr double m_fuel = 1.0;
static constexpr double I1 = 0.1;
static constexpr double I2 = 0.1;
static constexpr double I3 = 0.1;
static constexpr double Isp = 250.0;
} // namespace vprop

namespace sim {
static constexpr double h = 0.1;
static constexpr int num_states_6DOF = 13;
static constexpr int num_states_3DOF = 6;
} // namespace sim

namespace environment {
static constexpr double g = 9.81;
static constexpr double g0 = 9.81;
} // namespace environment