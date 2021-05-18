#include "nsim/ProcessNoise.hpp"

void ProcessNoise(Vector3d &F, Vector3d &M){
  F << F(0, 0) + random(0, process_noise::sig_Fx),
      F(1, 0) + random(0, process_noise::sig_Fy),
      F(2, 0) + random(0, process_noise::sig_Fz);
  M << M(0, 0) + random(0, process_noise::sig_Mx),
      M(1, 0) + random(0, process_noise::sig_My), M(2, 0);
}