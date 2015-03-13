/*
  CalibratedMag.h - CalibratedMag library for Wiring - implementation
  Copyright (c) 2015 Nate Costello.  Some rights reserved.
*/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

// include this library's description file
#include "CalibratedMag.h"

/** Default constructor.  Sets calibration parameters such that inputs are not modified
**/
CalibratedMag::CalibratedMag(void)
{
  mr11_ = 1;
  mr12_ = 0;
  mr13_ = 0;
  mr10_ = 0;
  mr21_ = 0;
  mr22_ = 1;
  mr23_ = 0;
  mr20_ = 0;
  mr31_ = 0;
  mr32_ = 0;
  mr33_ = 1;
  mr30_ = 0;
}

/**  Constructor.  Sets calibration parameters to the parameters provided.
@param mr11 - see ST Application Note AN3192
@param mr12 - see ST Application Note AN3192
@param mr13 - see ST Application Note AN3192
@param mr10 - see ST Application Note AN3192
@param mr21 - see ST Application Note AN3192
@param mr22 - see ST Application Note AN3192
@param mr23 - see ST Application Note AN3192
@param mr20 - see ST Application Note AN3192
@param mr31 - see ST Application Note AN3192
@param mr32 - see ST Application Note AN3192
@param mr33 - see ST Application Note AN3192
@param mr30 - see ST Application Note AN3192
**/
CalibratedMag::CalibratedMag(
  float mr11, float mr12, float mr13, float mr10, 
  float mr21, float mr22, float mr23, float mr20, 
  float mr31, float mr32, float mr33, float mr30){
  mr11_ = mr11;
  mr12_ = mr12;
  mr13_ = mr13;
  mr10_ = mr10;
  mr21_ = mr21;
  mr22_ = mr22;
  mr23_ = mr23;
  mr20_ = mr20;
  mr31_ = mr31;
  mr32_ = mr32;
  mr33_ = mr33;
  mr30_ = mr30;
}

/** Calibrate the mag field data
@param x int16_t container for the x-axis mag fields
@param y int16_t container for the y-axis mag fields
@param z int16_t container for the z-axis mag fields
**/
void CalibratedMag::calibrateMagFields(int16_t* x, int16_t* y, int16_t* z)
{
  float x_f = (int16_t)*x;
  float y_f = (int16_t)*y;
  float z_f = (int16_t)*z;

  x_f = x_f - mr10_;
  y_f = y_f - mr20_;
  z_f = z_f - mr30_;
  
  float xcal = x_f * mr11_ + y_f * mr12_ + z_f * mr13_;
  float ycal = x_f * mr21_ + y_f * mr22_ + z_f * mr23_;
  float zcal = x_f * mr31_ + y_f * mr32_ + z_f * mr33_;
  
  *x = (int16_t)xcal;
  *y = (int16_t)ycal;
  *z = (int16_t)zcal;

  // int x_int = (int16_t)x;
  // int y_int = (int16_t)y;
  // int z_int = (int16_t)z;

  // *x = x_int * 2;
  // *y = y_int * 2;
  // *z = z_int * 2;



}
