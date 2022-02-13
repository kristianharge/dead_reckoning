/**
 * @Author: Kristian Harge
 * @Date:   2022-02-11T21:33:50+01:00
 * @Email:  kristian.harge@yahoo.com
 * @Filename: libraries_mockup.cpp
 * @Last modified time: 2022-02-11T21:48:46+01:00
 */

#include <array>

//~ Function : gyrometerAcq
//~ ----------------------------
//~ Mockup of the gyrometer acquisition function
//~ inout : float &yawRate, is the returned yaw rate in rads/s, uint64_t &timestamp
//~  is the time in miliseconds at which the yaw rate acquisition was taken
//~
//~ output : int is 1 if suceess, -1 if error
int gyrometerAcq(float &yawRate, uint64_t &timestamp);

//~ Function : odometryAcq
//~ ----------------------------
//~ Mockup of the odometry acquisition function
//~ inout : std::array<float, 4> &odometry, is the returned wheel odometry in
//~   meters with the following order :
//~           [left_back, right_back, left_front, right_front]
//~   uint64_t &timestamp is the time in miliseconds at which the wheel odometry
//~     was taken
//~
//~ output : int is 1 if suceess, -1 if error
int odometryAcq(std::array<float, 4> &odometry, uint64_t &timestamp);
