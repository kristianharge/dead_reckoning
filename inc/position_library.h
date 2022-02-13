/**
 * @Author: Kristian Harge
 * @Date:   2022-02-11T21:33:50+01:00
 * @Email:  kristian.harge@yahoo.com
 * @Filename: position_library.h
 * @Last modified time: 2022-02-13T00:13:21+01:00
 */

#include <array>

#define MEAN(a, b)              (((a) + (b))/2)

#define XY_COORDS_SIZE             2
#define COORDS_SIZE                3
#define PI                         3.141592654

//~ Class: robotPosition
//~ ----------------------------
//~ Contains the robot positionning functions and information
class robotPosition{
  public:
    //contains the whole coordinates as : [x, y, tetha]
    std::array<float, COORDS_SIZE> coords = {0, 0, 0};

    robotPosition(void);
    ~robotPosition(void);
    void updateCoordsThreads(int gyroFreqHz, int odometryFreqHz);
    void updateGyroLoop(int gyroFreqHz);
    void updateXYLoop(int odometryFreqHz);

  private:

    void updateCoords(std::array<float, 4> odometry, uint64_t odometryTSMS,
      float yawRate, uint64_t yawRateTSMS);
    void updateXY(std::array<float, 4> odometry, uint64_t odometryTSMS);
    void updateGyro(float yawRate, uint64_t yawRateTSMS);

    //~ Function: calculateDeltaDist
    //~ ----------------------------
    //~ Calculates the variation in distance from the wheel odometry
    //~
    //~ input: std::array<float, 4> odometry; the wheel odometry array organized
    //~   as following : [left_back, right_back, left_front, right_front]
    //~
    //~ output: float; distance draveled by the point in between the two rear wheels
    float calculateDeltaDist(std::array<float, 4> odometry);

    //~ Function: calculateDeltaTetha
    //~ ----------------------------
    //~ Calculates the variation of the direction with the yawRate and time
    //~
    //~ input: float yawRate; the yaw rate in rad/s, uint64_t deltaTMs; the time
    //~   difference between the last yawRate acquisition and the new one
    //~
    //~ output: float; angle difference between the last position and the new one
    float calculateDeltaTetha(float yawRate, uint64_t deltaTMs);

    //~ Function: calculateTetha
    //~ ----------------------------
    //~ Calculates the new angle between our x axis and the robot direction
    //~
    //~ input: float deltaTetha; the angle variation in rads, float lastTetha;
    //~   the angle between x axis and our robot at the last positon
    //~
    //~ output: float; angle between x axis and our robot's current position
    float calculateTetha(float deltaTetha, float lastTetha);

    //~ Function: calculateDeltaCoords
    //~ ----------------------------
    //~ Calculates the x and y shift on the abolute coordinate system
    //~
    //~ input: float dist; distance traveled, float tetha;
    //~   the angle between x axis and our robot at the current positon
    //~
    //~ output: std::array<float, XY_COORDS_SIZE>; the x and y shift as following:
    //~   [x, y]
    std::array<float, XY_COORDS_SIZE> calculateDeltaCoords(float dist, float tetha);

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
    std::array<float, XY_COORDS_SIZE> getAbsCoords(
      std::array<float, XY_COORDS_SIZE> deltaCoords,
      std::array<float, XY_COORDS_SIZE> lastCoords);
};
