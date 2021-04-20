#pragma once
namespace FlightComputer {
static constexpr double dt = 0.1;
}
namespace Attitude {
static constexpr double K_p = 1.0;
static constexpr double K_i = 0;
static constexpr double K_d = 0;
} // namespace Attitude
namespace Position {
static constexpr double K_p = 1.0;
static constexpr double K_i = 0;
static constexpr double K_d = 0;
} // namespace Position