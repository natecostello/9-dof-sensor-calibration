/*
  CalibratedGyro.h 
  Copyright (c) 2015 Nate Costello.  All right reserved.

  calibration constants follow the convention of [insert datasheet]
*/

// ensure this library description is only included once
#ifndef CalibratedGyro_h
#define CalibratedGyro_h

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

// library interface description
class CalibratedGyro
{
  // user-accessible "public" interface
  public:
    CalibratedGyro();
    CalibratedGyro(
      float gcc11, float gcc12, float gcc13, float gcc10, 
      float gcc21, float gcc22, float gcc23, float gcc20, 
      float gcc31, float gcc32, float gcc33, float gcc30);

    void calibrateAngularVelocities(int16_t* x, int16_t* y, int16_t* z);

  // library-accessible "private" interface
  private:
    float gcc11_;
    float gcc12_;
    float gcc13_;
    float gcc10_;
    float gcc21_;
    float gcc22_;
    float gcc23_;
    float gcc20_;
    float gcc31_;
    float gcc32_;
    float gcc33_;
    float gcc30_;
};

#endif

