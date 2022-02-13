/**
 * @Author: Kristian Harge
 * @Date:   2022-02-11T22:31:55+01:00
 * @Email:  kristian.harge@yahoo.com
 * @Filename: tests.h
 * @Last modified time: 2022-02-13T00:14:34+01:00
 */



#define private public

#include <thread>

#include "position_library.h"

#include "CppUTest/TestHarness.h"
#include "CppUTest/CommandLineTestRunner.h"

using namespace std;

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
/////////////////////////extern functions///////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////test group//////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

TEST_GROUP(functional_tests)
{
  robotPosition robot_position;
  void setup()
   {
   }
   void teardown()
   {
   }
};

TEST_GROUP(robustness_tests)
{
  robotPosition robot_position;
  void setup()
   {
   }
   void teardown()
   {
   }
};

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////functionnal test functions//////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, getDistFromOdometry0){
  std::array<float, 4> odometry = {0, 0, 12, 10};
  float distanceResult = robot_position.calculateDeltaDist(odometry);

  CHECK_EQUAL(0, distanceResult);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, getDistFromOdometry10){
  std::array<float, 4> odometry = {1, 0, 5, 24};
  float distanceResult = robot_position.calculateDeltaDist(odometry);

  CHECK_EQUAL(0.5, distanceResult);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, getDistFromOdometryFloat){
  std::array<float, 4> odometry = {1.1, 0.3, 12.2, 24.1};
  float distanceResult = robot_position.calculateDeltaDist(odometry);

  DOUBLES_EQUAL((float) 0.7, distanceResult, 0.000001);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, getDistFromOdometryNegative){
  std::array<float, 4> odometry = {1.0, -1.0, 0, 0};
  float distanceResult = robot_position.calculateDeltaDist(odometry);

  CHECK_EQUAL(0, distanceResult);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, calculateDeltaTetha0){
  float yawRate = 0;
  uint32_t deltaTMs = 0;
  float deltaTetha = robot_position.calculateDeltaTetha(yawRate, deltaTMs);

  CHECK_EQUAL(0, deltaTetha);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, calculateDeltaTetha21){
  float yawRate = 2;
  uint32_t deltaTMs = 1;
  float deltaTetha = robot_position.calculateDeltaTetha(yawRate, deltaTMs);

  DOUBLES_EQUAL(0.002, deltaTetha, 0.000000001);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, calculateTetha0){
  float lastTetha = 0;
  float deltaTetha = 0;
  float tetha = robot_position.calculateTetha(deltaTetha, lastTetha);

  DOUBLES_EQUAL(0, tetha, 0.000000001);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, calculateTethaNegative){
  float lastTetha = 1.5;
  float deltaTetha = -2.5;
  float tetha = robot_position.calculateTetha(deltaTetha, lastTetha);

  DOUBLES_EQUAL(-1, tetha, 0.000000001);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, getDeltaCoords0){
  float dist = 0;
  float tetha = 0;

  std::array<float, XY_COORDS_SIZE> deltaCoords = robot_position.calculateDeltaCoords(dist, tetha);

  DOUBLES_EQUAL(0, deltaCoords[0], 0.000000001);
  DOUBLES_EQUAL(0, deltaCoords[1], 0.000000001);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, calculateDeltaCoords){
  float dist = 0.0051;
  float tetha = 0.856;

  std::array<float, XY_COORDS_SIZE> deltaCoords = robot_position.calculateDeltaCoords(dist, tetha);

  DOUBLES_EQUAL(0.003342864, deltaCoords[0], 0.000000001);
  DOUBLES_EQUAL(0.003851656, deltaCoords[1], 0.000000001);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, getDeltaCoordsNegativeAngle){
  float dist = 0.0051;
  float tetha = -0.856;

  std::array<float, XY_COORDS_SIZE> deltaCoords = robot_position.calculateDeltaCoords(dist, tetha);

  DOUBLES_EQUAL(0.003342864, deltaCoords[0], 0.000000001);
  DOUBLES_EQUAL(-0.003851656, deltaCoords[1], 0.000000001);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, getDeltaCoordsNegativeDist){
  float dist = -0.0051;
  float tetha = 0.856;

  std::array<float, XY_COORDS_SIZE> deltaCoords = robot_position.calculateDeltaCoords(dist, tetha);

  DOUBLES_EQUAL(-0.003342864, deltaCoords[0], 0.000000001);
  DOUBLES_EQUAL(-0.003851656, deltaCoords[1], 0.000000001);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, getDeltaCoordsNegativeDistAngle){
  float dist = -0.0051;
  float tetha = -0.856;

  std::array<float, XY_COORDS_SIZE> deltaCoords = robot_position.calculateDeltaCoords(dist, tetha);

  DOUBLES_EQUAL(-0.003342864, deltaCoords[0], 0.000000001);
  DOUBLES_EQUAL(0.003851656, deltaCoords[1], 0.000000001);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, getAbsCoords0){
  std::array<float, XY_COORDS_SIZE> deltaCoords = {0, 0};
  std::array<float, XY_COORDS_SIZE> lastCoords = {0, 0};

  std::array<float, XY_COORDS_SIZE> absCoords = robot_position.getAbsCoords(
    deltaCoords, lastCoords);

  DOUBLES_EQUAL(0, absCoords[0], 0.000000001);
  DOUBLES_EQUAL(0, absCoords[1], 0.000000001);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, getAbsCoords){
  std::array<float, XY_COORDS_SIZE> deltaCoords = {2.123, 4};
  std::array<float, XY_COORDS_SIZE> lastCoords = {4, 1.5};

  std::array<float, XY_COORDS_SIZE> absCoords = robot_position.getAbsCoords(
    deltaCoords, lastCoords);

  DOUBLES_EQUAL(6.123, absCoords[0], 0.000001);
  DOUBLES_EQUAL(5.5, absCoords[1], 0.000000001);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, getAbsCoordsNegative){
  std::array<float, XY_COORDS_SIZE> deltaCoords = {-2.123, -4};
  std::array<float, XY_COORDS_SIZE> lastCoords = {-4, -1.5};

  std::array<float, XY_COORDS_SIZE> absCoords = robot_position.getAbsCoords(
    deltaCoords, lastCoords);

  DOUBLES_EQUAL(-6.123, absCoords[0], 0.000001);
  DOUBLES_EQUAL(-5.5, absCoords[1], 0.000000001);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, getAbsCoordsNegativeDelta){
  std::array<float, XY_COORDS_SIZE> deltaCoords = {-2.123, -4};
  std::array<float, XY_COORDS_SIZE> lastCoords = {4, 1.5};

  std::array<float, XY_COORDS_SIZE> absCoords = robot_position.getAbsCoords(
    deltaCoords, lastCoords);

  DOUBLES_EQUAL(1.877, absCoords[0], 0.000001);
  DOUBLES_EQUAL(-2.5, absCoords[1], 0.000000001);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, getAbsCoordsNegativeLast){
  std::array<float, XY_COORDS_SIZE> deltaCoords = {2.123, 4};
  std::array<float, XY_COORDS_SIZE> lastCoords = {-4, -1.5};

  std::array<float, XY_COORDS_SIZE> absCoords = robot_position.getAbsCoords(
    deltaCoords, lastCoords);

  DOUBLES_EQUAL(-1.877, absCoords[0], 0.000001);
  DOUBLES_EQUAL(2.5, absCoords[1], 0.000000001);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(functional_tests, updateCoordsTest){
  DOUBLES_EQUAL(0, robot_position.coords[0], 0.000001);
  DOUBLES_EQUAL(0, robot_position.coords[1], 0.000001);
  DOUBLES_EQUAL(0, robot_position.coords[2], 0.000001);

  std::array<float, 4> odometry = {1, 1, 1, 1};
  uint32_t odometryTSMS = 500;
  float yawRate = 0;
  uint32_t yawRateTSMS = 500;

  robot_position.updateCoords(odometry, odometryTSMS, yawRate, yawRateTSMS);

  DOUBLES_EQUAL(1, robot_position.coords[0], 0.000001);
  DOUBLES_EQUAL(0, robot_position.coords[1], 0.000001);
  DOUBLES_EQUAL(0, robot_position.coords[2], 0.000001);

  odometry = {0.587, 0.413, 0.587, 0.413};
  odometryTSMS = 500;
  yawRate = 0.34906585;
  yawRateTSMS = 500;

  robot_position.updateCoords(odometry, odometryTSMS, yawRate, yawRateTSMS);

  DOUBLES_EQUAL(1.4924, robot_position.coords[0], 0.0001);
  DOUBLES_EQUAL(0.0868, robot_position.coords[1], 0.0001);
  DOUBLES_EQUAL(0.1745, robot_position.coords[2], 0.0001);

  odometry = {1, 1, 1, 1};
  odometryTSMS = 500;
  yawRate = 2.792526803;
  yawRateTSMS = 500;

  robot_position.updateCoords(odometry, odometryTSMS, yawRate, yawRateTSMS);

  DOUBLES_EQUAL(1.4924, robot_position.coords[0], 0.0001);
  DOUBLES_EQUAL(1.0868, robot_position.coords[1], 0.0001);
  DOUBLES_EQUAL(PI/2, robot_position.coords[2], 0.0001);

  odometry = {1.42, 1.42, 1.42, 1.42};
  odometryTSMS = 500;
  yawRate = -PI;
  yawRateTSMS = 500;

  robot_position.updateCoords(odometry, odometryTSMS, yawRate, yawRateTSMS);

  DOUBLES_EQUAL(2.9124, robot_position.coords[0], 0.0001);
  DOUBLES_EQUAL(1.0868, robot_position.coords[1], 0.0001);
  DOUBLES_EQUAL(0, robot_position.coords[2], 0.0001);

  odometry = {0.0255, 0.375, 0.0255, 0.375};
  odometryTSMS = 500;
  yawRate = -0.698131701;
  yawRateTSMS = 500;

  robot_position.updateCoords(odometry, odometryTSMS, yawRate, yawRateTSMS);

  DOUBLES_EQUAL(3.1005, robot_position.coords[0], 0.0001);
  DOUBLES_EQUAL(1.0183, robot_position.coords[1], 0.0001);
  DOUBLES_EQUAL(-0.3490, robot_position.coords[2], 0.0001);

  odometry = {1, -1, 1, -1};
  odometryTSMS = 500;
  yawRate = 4*PI;
  yawRateTSMS = 500;

  robot_position.updateCoords(odometry, odometryTSMS, yawRate, yawRateTSMS);

  DOUBLES_EQUAL(3.1005, robot_position.coords[0], 0.0001);
  DOUBLES_EQUAL(1.0183, robot_position.coords[1], 0.0001);
  DOUBLES_EQUAL(-0.3490, robot_position.coords[2], 0.0001);
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
/////////////////////robustness test functions//////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

//~ Test :
//~ ----------------------------
//~
//~
TEST(robustness_tests, calculateDeltaTethaOver2pi){
  float yawRate = 42;
  uint32_t deltaTMs = 1000;
  float deltaTetha = robot_position.calculateDeltaTetha(yawRate, deltaTMs);

  DOUBLES_EQUAL(4.30088815692, deltaTetha, 0.000001);
}

//~ Test :
//~ ----------------------------
//~
//~
TEST(robustness_tests, calculateTethaOver2pi){
  float lastTetha = PI;
  float deltaTetha = 2*PI;
  float tetha = robot_position.calculateTetha(deltaTetha, lastTetha);

  DOUBLES_EQUAL(PI, tetha, 0.000001);
}

int main(int ac, char** av)
{
    return CommandLineTestRunner::RunAllTests(ac, av);
}
