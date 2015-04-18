/*
  CalibratedAccel.h 
  Copyright (c) 2015 Nate Costello.  All right reserved.

  calibration constants follow the convention of [insert datasheet]
*/

// ensure this library description is only included once
#ifndef CalibratedAccel_h
#define CalibratedAccel_h

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

// library interface description
class CalibratedAccel
{
  // user-accessible "public" interface
  public:
    CalibratedAccel();
    CalibratedAccel(
      float acc11, float acc12, float acc13, float acc10, 
      float acc21, float acc22, float acc23, float acc20, 
      float acc31, float acc32, float acc33, float acc30);

    void calibrateAccelerations(float* x, float* y, float* z);

  // library-accessible "private" interface
  private:
    float acc11_;
    float acc12_;
    float acc13_;
    float acc10_;
    float acc21_;
    float acc22_;
    float acc23_;
    float acc20_;
    float acc31_;
    float acc32_;
    float acc33_;
    float acc30_;
};

#endif

