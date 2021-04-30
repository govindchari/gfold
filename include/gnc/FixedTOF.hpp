#pragma once
#include "Eigen/Dense"
#include "epigraph.hpp"
#include "gnc/constants.hpp"

using namespace cvx;
using namespace Eigen;

struct FixedTOFData {
  MatrixXd x_sol;
  MatrixXd u_sol;
  double m_final;
};

void FixedTOF(const Vector3d &r0, const Vector3d &v0, const double &tof,
              const double &m0, FixedTOFData &data);
