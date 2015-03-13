/*
  CalibratedMag.h 
  Copyright (c) 2015 Nate Costello.  All right reserved.

  calibration constants follow the convention of [insert datasheet]
*/

// ensure this library description is only included once
#ifndef CalibratedMag_h
#define CalibratedMag_h

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

// library interface description
class CalibratedMag
{
  // user-accessible "public" interface
  public:
    CalibratedMag();
    CalibratedMag(
      float mr11, float mr12, float mr13, float mr10, 
      float mr21, float mr22, float mr23, float mr20, 
      float mr31, float mr32, float mr33, float mr30);

    void calibrateMagFields(int16_t* x, int16_t* y, int16_t* z);

  // library-accessible "private" interface
  private:
    float mr11_;
    float mr12_;
    float mr13_;
    float mr10_;
    float mr21_;
    float mr22_;
    float mr23_;
    float mr20_;
    float mr31_;
    float mr32_;
    float mr33_;
    float mr30_;
};

#endif

