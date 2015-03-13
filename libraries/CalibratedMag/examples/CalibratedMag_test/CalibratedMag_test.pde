// This sketch tests the CallibratedAccel library
// by Nate Costello <email>
// Created X March 2015

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>

// I2Cdev and LSM303DLHC must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <LSM303DLHC.h>

// This is the library to be tested
#include <CalibratedMag.h>

// Calibration constants from AccelCal.m
float mr11 = 0.957712;
float mr12 = -0.003173;
float mr13 = 0.013866;
float mr10 = 25.095890;
float mr21 = -0.003173;
float mr22 = 0.957918;
float mr23 = -0.008213;
float mr20 = -88.009155;
float mr31 = 0.013866;
float mr32 = -0.008213;
float mr33 = 0.993407;
float mr30 = -8.876343;

// default address is 105
// specific I2C address may be passed here
LSM303DLHC accelMag;

int16_t mx, my, mz;

CalibratedMag myCalibratedMag (
    mr11, mr12, mr13, mr10,
    mr21, mr22, mr23, mr20,
    mr31, mr32, mr33, mr30);

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelMag.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelMag.testConnection() ? "LSM303DLHC connection successful" : "LSM303DLHC connection failed");

    // set mag data rate to 220hz
    accelMag.setMagOutputDataRate(220);

    // set mag gain
    accelMag.setMagGain(1100);

    // enable mag
    accelMag.setMagMode(LSM303DLHC_MD_CONTINUOUS);
}

void loop()
{
    // read raw mag field measurements from device
    accelMag.getMag(&mx, &my, &mz);

    // print uncalibrated data
    Serial.print("Raw MagField:\t");
    Serial.print(mx); Serial.print("\t");
    Serial.print(my); Serial.print("\t");
    Serial.print(mz); Serial.print("\t");

    //calibrate:
    myCalibratedMag.calibrateMagFields(&mx, &my, &mz);

    // print calibrated mag field measurements
    Serial.print("Cal MagField:\t");
    Serial.print(mx); Serial.print("\t");
    Serial.print(my); Serial.print("\t");
    Serial.print(mz); 
    Serial.println();

    delay(1000);
}

