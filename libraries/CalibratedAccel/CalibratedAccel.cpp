/*
  CalibratedAccel.h - CalibratedAccel library for Wiring - implementation
  Copyright (c) 2015 Nate Costello.  Some rights reserved.
*/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

// include this library's description file
#include "CalibratedAccel.h"

/** Default constructor.  Sets calibration parameters such that inputs are not modified
**/
CalibratedAccel::CalibratedAccel(void)
{
  acc11_ = 1;
  acc12_ = 0;
  acc13_ = 0;
  acc10_ = 0;
  acc21_ = 0;
  acc22_ = 1;
  acc23_ = 0;
  acc20_ = 0;
  acc31_ = 0;
  acc32_ = 0;
  acc33_ = 1;
  acc30_ = 0;
}

/**  Constructor.  Sets calibration parameters to the parameters provided.
@param acc11 - see ST Application Note AN3192
@param acc12 - see ST Application Note AN3192
@param acc13 - see ST Application Note AN3192
@param acc10 - see ST Application Note AN3192
@param acc21 - see ST Application Note AN3192
@param acc22 - see ST Application Note AN3192
@param acc23 - see ST Application Note AN3192
@param acc20 - see ST Application Note AN3192
@param acc31 - see ST Application Note AN3192
@param acc32 - see ST Application Note AN3192
@param acc33 - see ST Application Note AN3192
@param acc30 - see ST Application Note AN3192
**/
CalibratedAccel::CalibratedAccel(
  float acc11, float acc12, float acc13, float acc10, 
  float acc21, float acc22, float acc23, float acc20, 
  float acc31, float acc32, float acc33, float acc30){
  acc11_ = acc11;
  acc12_ = acc12;
  acc13_ = acc13;
  acc10_ = acc10;
  acc21_ = acc21;
  acc22_ = acc22;
  acc23_ = acc23;
  acc20_ = acc20;
  acc31_ = acc31;
  acc32_ = acc32;
  acc33_ = acc33;
  acc30_ = acc30;
}

/** Calibrate the acceleration data
@param x float container for the x-axis acceleration
@param y float container for the y-axis acceleration
@param z float container for the z-axis acceleration
**/
void CalibratedAccel::calibrateAccelerations(int16_t* x, int16_t* y, int16_t* z)
{
  float x_f = *x;
  float y_f = *y;
  float z_f = *z;

  float xcal = x_f * acc11_ + y_f * acc12_ + z_f * acc13_ + acc10_;
  float ycal = x_f * acc21_ + y_f * acc22_ + z_f * acc23_ + acc20_;
  float zcal = x_f * acc31_ + y_f * acc32_ + z_f * acc33_ + acc30_;
  
  *x = (int16_t)xcal;
  *y = (int16_t)ycal;
  *z = (int16_t)zcal;
}
