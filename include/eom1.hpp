#pragma once
#include "../lib/eigen/Eigen/Dense"
using namespace Eigen;

Vector2d eom1(const Vector2d state) {
    auto x = state.block<1, 1>(0, 0);
    auto v = state.block<1, 1>(1, 0);
    Vector2d state_d;
    state_d << v, -x;
    return state_d;
}