
#include "libraries_mockup.h"

int gyrometerAcq(float &yawRate, uint32_t &timestamp){
  return 1;
}

int odometryAcq(std::array<float, 4> &odometry, uint32_t &timestamp){
  return 1;
}
