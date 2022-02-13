/**
 * @Author: Kristian Harge
 * @Date:   2022-02-11T21:33:50+01:00
 * @Email:  kristian.harge@yahoo.com
 * @Filename: main.cpp
 * @Last modified time: 2022-02-13T00:13:30+01:00
 */

#include "position_library.h"

int main(){

  robotPosition robot_position;

  robot_position.updateCoordsThreads(100, 50);

  return 0;
}
