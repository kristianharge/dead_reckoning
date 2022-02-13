
#include "libraries_mockup.h"

int gyrometerAcq(float &yawRate, uint64_t &timestamp){
  return 1;
}

int odometryAcq(std::array<float, 4> &odometry, uint64_t &timestamp){
  return 1;
}
