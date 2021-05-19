#include "nsim/simulations/MonteCarlo.hpp"

void MonteCarlo(const int trials){
    double x0_mean = 0;
    double y0_mean = 0;
    double z0_mean = 2000;
    double vx0_mean = 0;
    double vy0_mean = 0;
    double vz0_mean = -50;
    double x0_std = 500;
    double y0_std = 500;
    double z0_std = 100;
    double vx0_std = 10;
    double vy0_std = 10;
    double vz0_std = 10;
    Matrix<double, sim::num_states_6DOF, 1> z0;  
    Vector3d r0;
    Vector4d q0;
    Vector3d v0;
    Vector3d w0;
    double m0;
    FixedTOFData data;

    for (int i=1; i <= trials; i++){
        r0 << random(x0_mean, x0_std), random(y0_mean, y0_std), random(z0_mean, z0_std);
        q0 << 1, 0, 0, 0;
        v0 << random(vx0_mean, vx0_std), random(vy0_mean, vy0_std), random(vz0_mean, vz0_std);
        w0 << 0, 0, 0;
        m0 = massprop::m_fuel + massprop::m_dry;
        z0<<r0, q0, v0, w0, m0;
        run6DOF(z0);
    }
}