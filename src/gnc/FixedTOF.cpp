#include "gnc/FixedTOF.hpp"

void FixedTOF(const Vector3d &r0, const Vector3d &v0, const double &tof,
              const double &m0, FixedTOFData &data) {
  MatrixXd x0(6, 1);
  MatrixXd xT(6, 1);
  MatrixXd c(1, 6);
  MatrixXd S(2, 6);
  MatrixXd A(6, 6);
  MatrixXd B(6, 3);
  MatrixXd G(6, 1);

  double a = propulsion::alph;
  double rho1 = propulsion::T_min;
  double rho2 = propulsion::T_max;
  double gs = position::glideslope;
  double dt = FlightComputer::dt;
  double g = environment::g;
  int time_horizon = (int) (tof / dt);
  double tvc = attitude::pointing_max;
  double mu1;
  double mu2;
  double z0;

  A << 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
  B << 0.5 * dt * dt, 0, 0, 0, 0.5 * dt * dt, 0, 0, 0, 0.5 * dt * dt, dt, 0, 0,
      0, dt, 0, 0, 0, dt;
  G << 0, 0, -0.5 * g * dt * dt, 0, 0, -g * dt;
  S << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
  c << 0, 0, -tan(M_PI_2 - gs), 0, 0, 0;
  x0 << r0, v0;
  xT << 0, 0, 0, 0, 0, 0;

  OptimizationProblem socp;
  size_t T = time_horizon;
  VectorX z = socp.addVariable("z", T + 1);
  Scalar q = socp.addVariable("q");
  MatrixX x = socp.addVariable("x", 6, T + 1);
  MatrixX u = socp.addVariable("u", 3, T + 1);
  VectorX s = socp.addVariable("s", T + 1);

  // Initial and Terminal Constraints
  socp.addConstraint(equalTo(x.col(0), par(x0)));
  socp.addConstraint(lessThan(x.col(T)(2), 0.4));
  socp.addConstraint(lessThan(sqrt(x.col(T)(3)*x.col(T)(3)+x.col(T)(4)*x.col(T)(4)+x.col(T)(5)*x.col(T)(5)), 1.0));
  socp.addConstraint(equalTo(z(0), log(m0)));


  // Dynamics
  for (int k = 0; k < T; k++) {
    socp.addConstraint(
        equalTo(x.col(k + 1), par(A) * x.col(k) + par(B) * u.col(k) + par(G)));
    socp.addConstraint(equalTo(z(k + 1), z(k) - a * s(k) * dt));
  }

  socp.addConstraint(greaterThan(q, sqrt(x.col(T)(0)*x.col(T)(0)+x.col(T)(1)*x.col(T)(1))));


  // State and Input Constraints
  for (int k = 0; k < T + 1; k++) {
    z0 = log(m0 - (a * rho2 * (k) * dt));
    mu1 = rho1 * exp(-z0);
    mu2 = rho2 * exp(-z0);
    socp.addConstraint(lessThan(u.col(k).norm(), s(k)));
    socp.addConstraint(lessThan(mu1 * (1.0 - (z(k) - z0)), s(k)));
    socp.addConstraint(lessThan(s(k), mu2 * (1.0 - (z(k) - z0))));
    socp.addConstraint(lessThan(log(m0 - a * rho2 * k * dt), z(k)));
    socp.addConstraint(lessThan(z(k), log(m0 - a * rho1 * k * dt)));
    if (r0(2,0) >= 300.0){
      socp.addConstraint(lessThan((par(S) * x.col(k)).norm(), -par(c) * x.col(k)));
    }
    socp.addConstraint(greaterThan(z(k), 0));
    socp.addConstraint(greaterThan(x.col(k)(2), 0));
    socp.addConstraint(greaterThan(u.col(k)(2), s(k) * cos(tvc)));
  }

  // Cost
  socp.addCostTerm(-z(T) + 0.001 * q);

  // Create and initialize the solver instance.
  ecos::ECOSSolver solver(socp);

  // Solve problem and show solver output
  const bool verbose = false;
  solver.solve(verbose);
  auto x_sol = eval(x);
  auto u_sol = eval(u);
  auto s_sol = eval(s);
  auto z_sol = eval(z);

  data.x_sol = x_sol;
  data.u_sol = u_sol;
  data.z_sol = z_sol;
  data.m_final = exp(z_sol(T));

  if (!(solver.getExitCode() == 0 || solver.getExitCode() == 10)) {
    data.m_final = massprop::m_dry + massprop::m_fuel;
  }
}