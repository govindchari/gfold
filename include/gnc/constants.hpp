#pragma once
namespace FlightComputer {
static constexpr double dt = 1;
}
namespace massprop {
static constexpr double m_dry = 25600.0;
static constexpr double m_fuel = 10000.0;
static constexpr double l = 20.0;
static constexpr double r = 1.85928;
static constexpr double I1 = 0.25*(m_dry+m_fuel)*r*r+(1.0/12.0)*(m_dry+m_fuel)*(2*l*2*l);
static constexpr double I2 = 0.25*(m_dry+m_fuel)*r*r+(1.0/12.0)*(m_dry+m_fuel)*(2*l*2*l);
static constexpr double I3 = 0.5*(m_dry+m_fuel)*r*r;
} // namespace massprop

namespace environment {
static constexpr double g = 9.807;
static constexpr double g0 = 9.807;
} // namespace environment
namespace propulsion {
static constexpr double Isp = 311.0;
static constexpr double T_min = 0.40 * 411000.0;
static constexpr double T_max = 411000.0;
static constexpr double alph = 1.0 / (propulsion::Isp * environment::g0);
} // namespace propulsion

namespace attitude {
static constexpr double K_p = massprop::I1;
static constexpr double K_i = 0;
static constexpr double K_d = 0.6 * K_p;
static constexpr double theta_max = 15.0 * (M_PI / 180.0);
static constexpr double pointing_max = 25.0 * (M_PI / 180.0);
} // namespace attitude
namespace position {
static constexpr double glideslope = 1.0 * (M_PI/180);
static constexpr double tolerance = 0.5;
} // namespace position