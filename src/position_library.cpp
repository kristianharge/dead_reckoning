/**
 * @Author: Kristian Harge
 * @Date:   2022-02-11T21:33:50+01:00
 * @Email:  kristian.harge@yahoo.com
 * @Filename: position_library.cpp
 * @Last modified time: 2022-02-13T00:13:21+01:00
 */

#include <cmath>
#include <array>
#include <chrono>
#include <thread>
#include <iostream>

#include "libraries_mockup.h"

#include "position_library.h"

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
///////////////////////constructor destructor///////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

robotPosition::robotPosition(void){
}

robotPosition::~robotPosition(void){
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
///////////////////////////public methods///////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void robotPosition::updateCoordsThreads(int gyroFreqHz, int odometryFreqHz){
  //create the gyroscope update thread
  std::thread gyroThread(&robotPosition::updateGyroLoop, this, gyroFreqHz);
  //create the odometry and xy position thread
  std::thread XYThread(&robotPosition::updateXYLoop, this, odometryFreqHz);

  gyroThread.join();
  XYThread.join();
}

void robotPosition::updateGyroLoop(int gyroFreqHz){
  float yawRate = 0.0;
  uint64_t timestamp = 0;
  int ret = 1;
  using std::chrono::high_resolution_clock;
  using std::chrono::duration_cast;
  using std::chrono::duration;
  using std::chrono::milliseconds;
  auto start = high_resolution_clock::now();

  while(1){
    ret = gyrometerAcq(yawRate, timestamp);
    if (ret > 0){
      updateGyro(yawRate, timestamp);
      auto end = high_resolution_clock::now();
      auto timeElapsed = duration_cast<milliseconds>(end - start);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000/gyroFreqHz - timeElapsed.count()));
      start = high_resolution_clock::now();

#ifdef DEBUG
      std::cout << "yaw rate : " << yawRate <<", timestamp : " << timestamp <<
        ", loop period : " << 1000/gyroFreqHz -
        std::chrono::duration_cast<std::chrono::milliseconds>(timeElapsed).count()
        << std::endl;
#endif
    }
  }
}

void robotPosition::updateXYLoop(int odometryFreqHz){
  std::array<float, 4> odometry;
  uint64_t timestamp = 0;
  int ret = 1;
  using std::chrono::high_resolution_clock;
  using std::chrono::duration_cast;
  using std::chrono::duration;
  using std::chrono::milliseconds;
  auto start = high_resolution_clock::now();

  while(1){
    ret = odometryAcq(odometry, timestamp);
    if (ret > 0){
      updateXY(odometry, timestamp);
      auto end = high_resolution_clock::now();
      auto timeElapsed = duration_cast<milliseconds>(end - start);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000/odometryFreqHz - timeElapsed.count()));
      start = high_resolution_clock::now();

#ifdef DEBUG
      std::cout << "odometry : " << odometry[0] << " " << odometry[1] <<
        ", timestamp : " << timestamp << ", loop period : " << 1000/odometryFreqHz -
        std::chrono::duration_cast<std::chrono::milliseconds>(timeElapsed).count()
        << std::endl;
#endif
    }
  }
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
///////////////////////////private methods//////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void robotPosition::updateCoords(std::array<float, 4> odometry,
  uint64_t odometryTSMS, float yawRate, uint64_t yawRateTSMS){

  updateGyro(yawRate, yawRateTSMS);
  updateXY(odometry, odometryTSMS);
}

void robotPosition::updateXY(std::array<float, 4> odometry,
  uint64_t odometryTSMS){

  std::array<float, XY_COORDS_SIZE> xyLastCoords = {coords[0], coords[1]};
  float tetha = coords[2];
  float deltaDist = calculateDeltaDist(odometry);
  std::array<float, XY_COORDS_SIZE> deltaCoords =
    calculateDeltaCoords(deltaDist, tetha);
  std::array<float, XY_COORDS_SIZE> absCoords =
    getAbsCoords(deltaCoords, xyLastCoords);

  coords[0] = absCoords[0];
  coords[1] = absCoords[1];
}

void robotPosition::updateGyro(float yawRate, uint64_t yawRateTSMS){

  float lastTetha = coords[2];
  float deltaTetha = calculateDeltaTetha(yawRate, yawRateTSMS);
  float tetha = calculateTetha(deltaTetha, lastTetha);
  coords[2] = tetha;
}

//~ Function: calculateDeltaDist
//~ ----------------------------
//~ Calculates the variation in distance from the wheel odometry
//~
//~ input: std::array<float, 4> odometry; the wheel odometry array organized
//~   as following : [left_back, right_back, left_front, right_front]
//~
//~ output: float; distance draveled by the point in between the two rear wheels
float robotPosition::calculateDeltaDist(std::array<float, 4> odometry){
  return MEAN(odometry[0], odometry[1]);
}// end function calculateDeltaDist

//~ Function: calculateDeltaTetha
//~ ----------------------------
//~ Calculates the variation of the direction with the yawRate and time
//~
//~ input: float yawRate; the yaw rate in rad/s, uint64_t deltaTMs; the time
//~   difference between the last yawRate acquisition and the new one
//~
//~ output: float; angle difference between the last position and the new one
float robotPosition::calculateDeltaTetha(float yawRate, uint64_t deltaTMs){
  float deltaTS = (float) deltaTMs/1000;
  return fmod(deltaTS*yawRate, 2*PI);
}// end function calculateDeltaTetha

//~ Function: calculateTetha
//~ ----------------------------
//~ Calculates the new angle between our x axis and the robot direction
//~
//~ input: float deltaTetha; the angle variation in rads, float lastTetha;
//~   the angle between x axis and our robot at the last positon
//~
//~ output: float; angle between x axis and our robot's current position
float robotPosition::calculateTetha(float deltaTetha, float lastTetha){
  return fmod((deltaTetha + lastTetha), 2*PI);
}// end function calculateTetha

//~ Function: calculateDeltaCoords
//~ ----------------------------
//~ Calculates the x and y shift on the abolute coordinate system
//~
//~ input: float dist; distance traveled, float tetha;
//~   the angle between x axis and our robot at the current positon
//~
//~ output: std::array<float, XY_COORDS_SIZE>; the x and y shift as following:
//~   [x, y]
std::array<float, XY_COORDS_SIZE> robotPosition::calculateDeltaCoords
  (float dist, float tetha){

  std::array<float, XY_COORDS_SIZE> deltaCoords;
  deltaCoords[0] = dist*cos(tetha);
  deltaCoords[1] = dist*sin(tetha);

  return deltaCoords;
}// end function calculateDeltaCoords

//~ Function: getAbsCoords
//~ ----------------------------
//~ Calculates the current x and y on the absolute coordinate system
//~
//~ input: std::array<float, XY_COORDS_SIZE> deltaCoords; x and y shift from
//~   the last position, std::array<float, XY_COORDS_SIZE> lastCoords; x and y
//~   of the last position
//~
//~ output: std::array<float, XY_COORDS_SIZE>; x and y on the absolute
//~   coordinate system as following: [x, y]
std::array<float, XY_COORDS_SIZE> robotPosition::getAbsCoords(
  std::array<float, XY_COORDS_SIZE> deltaCoords,
  std::array<float, XY_COORDS_SIZE> lastCoords){

  std::array<float, XY_COORDS_SIZE> absCoords;

  for(int i = 0; i < XY_COORDS_SIZE; i++){
    absCoords[i] = lastCoords[i] + deltaCoords[i];
  }

  return absCoords;
}// end function getAbsCoords
