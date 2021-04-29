#include <iostream>
#include <fstream>
#include "Eigen/Dense"
#include "epigraph.hpp"
using namespace cvx;
using namespace std;
using namespace Eigen;

void saveData(string fileName, MatrixXd  matrix)
{
    //https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
    const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");
 
    ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(CSVFormat);
        file.close();
    }
}

int main() {  
  Eigen::MatrixXd r0(3,1);
  Eigen::MatrixXd v0(3,1);
  Eigen::MatrixXd x0(6, 1);
  Eigen::MatrixXd rT(3,1);
  Eigen::MatrixXd vT(3,1);
  Eigen::MatrixXd xT(6, 1);
  Eigen::MatrixXd c(1,6);
  Eigen::MatrixXd S(2,6);
  Eigen::MatrixXd A(6, 6);
  Eigen::MatrixXd B(6, 3);
  Eigen::MatrixXd G(6, 1);

  double a = 1.0 / (255.0 * 9.807);
  double rho1 = 0.15 * 22000;
  double rho2 = 22000.0;
  double gs = 0.0349066;
  double dt = 0.1;
  double g = 3.711;
  double tof = 100.0;
  int time_horizon = (tof/dt);
  double mdry = 1500.0;
  double mfuel = 500.0;
  double mt = mdry + mfuel;
  double tvc = 0.436332313;
  double mu1;
  double mu2;
  double z0;
  double z_0;

  A << 1, 0, 0, dt, 0, 0,
       0, 1, 0, 0, dt, 0,
       0, 0, 1, 0, 0, dt,
       0, 0, 0, 1, 0, 0, 
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1;
  B << 0.5*dt*dt, 0, 0,
       0, 0.5*dt*dt, 0,
       0, 0, 0.5*dt*dt,
       dt, 0, 0, 
       0, dt, 0,
       0, 0, dt;
  G << 0, 0, -0.5*g*dt*dt, 0, 0, -g*dt;
  S << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0;
  c << 0, 0, -tan(M_PI_2 - gs), 0, 0, 0;
  r0 << 1500, 0, 2000;
  v0 << 70, 0, -75;
  rT << 0, 0, 0;
  vT << 0, 0, 0;
  x0 << r0, v0;
  xT << rT, vT;

  OptimizationProblem socp;
  size_t T = time_horizon;
  VectorX z = socp.addVariable("z", T + 1);
  MatrixX x = socp.addVariable("x", 6, T + 1);
  MatrixX u = socp.addVariable("u", 3, T + 1);
  VectorX s = socp.addVariable("s", T + 1);

  // Initial and Terminal Constraints
  socp.addConstraint(equalTo(x.col(0), par(x0)));
  socp.addConstraint(equalTo(x.col(T), par(xT)));
  socp.addConstraint(equalTo(z(0), log(mt)));

  // Dynamics
  for (int k=0; k<T; k++) {
    socp.addConstraint(equalTo(x.col(k+1), par(A) * x.col(k) + par(B) * u.col(k) + par(G)));
    socp.addConstraint(equalTo(z(k + 1), z(k) - a * s(k) * dt));
  }

  // State and Input Constraints
  for (int k=0; k<T+1; k++) {
    z0 = log(mt - (a * rho2 * (k) * dt));
    mu1 = rho1*exp(-z0);
    mu2 = rho2*exp(-z0);

    socp.addConstraint(lessThan(u.col(k).norm(), s(k)));
    socp.addConstraint(lessThan(mu1 * (1.0-(z(k)-z0)), s(k)));
    socp.addConstraint(lessThan(s(k), mu2 * (1.0 - (z(k) - z0))));
    socp.addConstraint(lessThan(log(mt - a * rho2 * k * dt), z(k)));
    socp.addConstraint(lessThan(z(k), log(mt - a * rho1 * k * dt)));
    socp.addConstraint(lessThan((par(S)*x.col(k)).norm(), -par(c) * x.col(k)));
    socp.addConstraint(greaterThan(z(k), 0));
    socp.addConstraint(greaterThan(x.col(k)(2), 0));
    socp.addConstraint(greaterThan(u.col(k)(2), s(k)*cos(tvc)));
  }
  

  // Cost
  socp.addCostTerm(-z(T));

  // Print the problem formulation for inspection
  std::cout << socp << "\n";
  // Create and initialize the solver instance.
  ecos::ECOSSolver solver(socp);

  // Print the canonical problem formulation for inspection
  //std::cout << solver << "\n";

  // Solve problem and show solver output
  const bool verbose = true;
  solver.solve(verbose);

  Eigen::MatrixXd x_sol = eval(x); 
  Eigen::MatrixXd u_sol = eval(u);
  Eigen::MatrixXd s_sol = eval(s);
  auto z_sol = eval(z);

  std::cout << -z_sol(T) << std::endl;
  saveData("pos", x_sol);
  saveData("mass", z_sol);

}

