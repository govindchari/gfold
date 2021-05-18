#include "gnc/FreeTOF.hpp"
#include <iostream>

Vector3d FreeTOF(const Vector3d &r0, const Vector3d &v0, const double &m0, double &tof_guess, FixedTOFData &data) {
    int t_min = (int) (massprop::m_dry) * v0.norm() / propulsion::T_max;
    int t_max = (int) (m0 - massprop::m_dry) / (propulsion::alph * propulsion::T_min);
    double i = t_min;
    if (tof_guess < t_max) {
        i = tof_guess - 10.0 * FlightComputer::dt;
        if (tof_guess < 15.0 * FlightComputer::dt) {
            i=0.0;
        }
    }

    while (i < t_max){
        FixedTOF(r0, v0, i, m0, data);
        if((data.m_final < massprop::m_dry + massprop::m_fuel)) {
            std::cout << i << std::endl;
            tof_guess = i - FlightComputer::dt;
            return m0 * data.u_sol.col(0);
        }
        i+=FlightComputer::dt;
    }
    Vector3d debug;
    debug << 69, 69, 69;
    return debug;
}