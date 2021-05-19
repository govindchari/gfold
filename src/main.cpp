#include <iostream>
#include <chrono> 
#include <fstream>
#include <string>
#include "Eigen/Dense"
#include "epigraph.hpp"
#include "nsim/simulations/run6DOF.hpp"
#include "nsim/simulations/MonteCarlo.hpp"
using namespace cvx;
using namespace std;
using namespace Eigen;
using namespace std::chrono;

void saveData(std::string fileName, MatrixXd  matrix)
{
    const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");
 
    ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(CSVFormat);
        file.close();
    }
}

int main() {
  MonteCarlo(200);
  /*
  Matrix<double, sim::num_states_6DOF, 1> z0;  
  Vector3d r0;
  Vector4d q0;
  Vector3d v0;
  Vector3d w0;
  double m0;
  FixedTOFData data;
  r0 << -2500, 0, 2000;
  q0 << 1, 0, 0, 0;
  v0 << 50, 70, -75;
  w0 << 0, 0, 0;
  m0 = massprop::m_fuel + massprop::m_dry;

  z0<<r0, q0, v0, w0, m0;

  auto start = high_resolution_clock::now();

  run6DOF(z0);
  
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(stop - start);
  std::cout << duration.count() << std::endl;

  FixedTOF(r0, v0, 43, m0, data);
  saveData("pos", data.x_sol);
  saveData("u", data.u_sol);
  saveData("z", data.z_sol);
  */
  
  
}

