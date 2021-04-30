#pragma once
namespace FlightComputer {
static constexpr double dt = 0.01;
}
namespace massprop {
static constexpr double m_dry = 1500.0;
static constexpr double m_fuel = 500.0;
static constexpr double l = 0.5;
static constexpr double I1 = 0.1;
static constexpr double I2 = 0.1;
static constexpr double I3 = 0.1;
} // namespace massprop

namespace propulsion {
static constexpr double Isp = 255.0;
static constexpr double T_min = 0.15 * 22000.0;
static constexpr double T_max = 22000.0;
} // namespace propulsion

namespace environment {
static constexpr double g = 3.711;
static constexpr double g0 = 9.807;
} // namespace environment

namespace attitude {
static constexpr double K_p = 10.0;
static constexpr double K_i = 0;
static constexpr double K_d = 1.0;
static constexpr double theta_max = 15.0 * (M_PI / 180.0);
static constexpr double pointing_max = 25.0*(M_PI/180.0);
} // namespace attitude
namespace position {
static constexpr double K_p = 0.01;
static constexpr double K_i = 0;
static constexpr double K_d = 0.0;
static constexpr double glideslope = 4.0*(M_PI/180);
} // namespace position