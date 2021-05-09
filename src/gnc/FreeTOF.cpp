#include "gnc/FreeTOF.hpp"
#include <iostream>

Vector3d FreeTOF(const Vector3d &r0, const Vector3d &v0, const double &m0, double &tof_guess, FixedTOFData &data) {
    int t_min = (int) (massprop::m_dry) * v0.norm() / propulsion::T_max;
    int t_max = (int) (m0 - massprop::m_dry) / (propulsion::alph * propulsion::T_min);
    double i = t_min;
    if (tof_guess < t_max) {
        i = tof_guess-FlightComputer::dt;
    }

    while (i < t_max){
        FixedTOF(r0, v0, i, m0, data);
        //std::cout << i << std::endl;
        if((data.m_final < massprop::m_dry + massprop::m_fuel)) {
            std::cout << i << std::endl;
            tof_guess = i - FlightComputer::dt;
            return m0 * data.u_sol.col(0);
        }
        i+=FlightComputer::dt;
    }
}