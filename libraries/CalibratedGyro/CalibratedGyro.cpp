/*
  CalibratedGyro.h - CalibratedGyro library for Wiring - implementation
  Copyright (c) 2015 Nate Costello.  Some rights reserved.
*/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

// include this library's description file
#include "CalibratedGyro.h"

/** Default constructor.  Sets calibration parameters such that inputs are not modified
**/
CalibratedGyro::CalibratedGyro(void)
{
  gcc11_ = 1;
  gcc12_ = 0;
  gcc13_ = 0;
  gcc10_ = 0;
  gcc21_ = 0;
  gcc22_ = 1;
  gcc23_ = 0;
  gcc20_ = 0;
  gcc31_ = 0;
  gcc32_ = 0;
  gcc33_ = 1;
  gcc30_ = 0;
}

/**  Constructor.  Sets calibration parameters to the parameters provided.
@param gcc11 - see ST Application Note AN3192
@param gcc12 - see ST Application Note AN3192
@param gcc13 - see ST Application Note AN3192
@param gcc10 - see ST Application Note AN3192
@param gcc21 - see ST Application Note AN3192
@param gcc22 - see ST Application Note AN3192
@param gcc23 - see ST Application Note AN3192
@param gcc20 - see ST Application Note AN3192
@param gcc31 - see ST Application Note AN3192
@param gcc32 - see ST Application Note AN3192
@param gcc33 - see ST Application Note AN3192
@param gcc30 - see ST Application Note AN3192
**/
CalibratedGyro::CalibratedGyro(
  float gcc11, float gcc12, float gcc13, float gcc10, 
  float gcc21, float gcc22, float gcc23, float gcc20, 
  float gcc31, float gcc32, float gcc33, float gcc30){
  gcc11_ = gcc11;
  gcc12_ = gcc12;
  gcc13_ = gcc13;
  gcc10_ = gcc10;
  gcc21_ = gcc21;
  gcc22_ = gcc22;
  gcc23_ = gcc23;
  gcc20_ = gcc20;
  gcc31_ = gcc31;
  gcc32_ = gcc32;
  gcc33_ = gcc33;
  gcc30_ = gcc30;
}

/** Calibrate the angular velocity data
@param x int16_t container for the x-axis mag fields
@param y int16_t container for the y-axis mag fields
@param z int16_t container for the z-axis mag fields
**/
void CalibratedGyro::calibrateAngularVelocities(int16_t* x, int16_t* y, int16_t* z)
{
  float x_f = (int16_t)*x;
  float y_f = (int16_t)*y;
  float z_f = (int16_t)*z;

  x_f = x_f - gcc10_;
  y_f = y_f - gcc20_;
  z_f = z_f - gcc30_;
  
  float xcal = x_f * gcc11_ + y_f * gcc12_ + z_f * gcc13_;
  float ycal = x_f * gcc21_ + y_f * gcc22_ + z_f * gcc23_;
  float zcal = x_f * gcc31_ + y_f * gcc32_ + z_f * gcc33_;
  
  *x = (int16_t)xcal;
  *y = (int16_t)ycal;
  *z = (int16_t)zcal;

}