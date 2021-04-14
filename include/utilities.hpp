#pragma once
#include "../lib/eigen/Eigen/Dense"
using namespace Eigen;

template <typename T>
T hamilton(const T q,const T r) {
    T result;
    result <<r(0,0)*q(0,0)-r(1,0)*q(1,0)-r(2,0)*q(2,0)-r(3,0)*q(3,0),
             r(0,0)*q(1,0)+r(1,0)*q(0,0)-r(2,0)*q(3,0)+r(3,0)*q(2,0),
             r(0,0)*q(2,0)+r(1,0)*q(3,0)+r(2,0)*q(0,0)-r(3,0)*q(1,0),
             r(0,0)*q(3,0)-r(1,0)*q(2,0)+r(2,0)*q(1,0)+r(3,0)*q(0,0);
    return result;
}