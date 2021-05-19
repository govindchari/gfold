#pragma once
namespace sim {
static constexpr double h = 0.1;
static constexpr int num_states_6DOF = 14;
} // namespace sim

namespace process_noise {
static constexpr double sig_Fx = 1000;
static constexpr double sig_Fy = 1000;
static constexpr double sig_Fz = 1000;
static constexpr double sig_Mx = 1000;
static constexpr double sig_My = 1000;
} // namespace process_noise
