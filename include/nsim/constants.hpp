#pragma once
namespace sim {
static constexpr double h = 0.1;
static constexpr int num_states_6DOF = 14;
} // namespace sim

namespace process_noise {
static constexpr double sig_Fx = 3000;
static constexpr double sig_Fy = 3000;
static constexpr double sig_Fz = 3000;
static constexpr double sig_Mx = 3000;
static constexpr double sig_My = 3000;
} // namespace process_noise
