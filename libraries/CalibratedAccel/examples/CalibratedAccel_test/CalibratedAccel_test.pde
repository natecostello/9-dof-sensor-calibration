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

// This is the library to be testing
#include <CalibratedAccel.h>

// Calibration constants from AccelCal.m
float acc11 = 1.007441;
float acc12 = -0.002214;
float acc13 = 0.002661;
float acc10 = 210.452155;
float acc21 = 0.006953;
float acc22 = 0.993671;
float acc23 = 0.001837;
float acc20 = 216.580785;
float acc31 = -0.004203;
float acc32 = -0.001992;
float acc33 = 1.004737;
float acc30 = -1046.996387;

// LSB scaling to G for +-2G range setting
float scale = 0.000061035;

// scaled accelerations
float sax;
float say;
float saz;

// stopwatch to time calibration
uint16_t stopwatch;

LSM303DLHC accelMag;

int16_t ax, ay, az;

CalibratedAccel myCalibratedAccel(
    acc11, acc12, acc13, acc10,
    acc21, acc22, acc23, acc20,
    acc31, acc32, acc33, acc30);

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

    // set scale to 2Gs
    accelMag.setAccelFullScale(2);

    // set high res output to get full 12-bit accuracy
    accelMag.setAccelHighResOutputEnabled(true);

    // set accel data rate to 200hz
    accelMag.setAccelOutputDataRate(200);
}

void loop()
{
    // read raw angular velocity measurements from device
    accelMag.getAcceleration(&ax, &ay, &az);
    sax = scale * ax;
    say = scale * ay;
    saz = scale * az;

    Serial.print("Raw Accelation:\t");
    Serial.print(sax); Serial.print("\t");
    Serial.print(say); Serial.print("\t");
    Serial.print(saz); Serial.print("\t");


    //calibrate and time calibration
    stopwatch = micros();
    myCalibratedAccel.calibrateAccelerations(&sax, &say, &saz);
    stopwatch = micros() - stopwatch;

    Serial.print("Cal Accelation:\t");
    Serial.print(sax); Serial.print("\t");
    Serial.print(say); Serial.print("\t");
    Serial.print(saz); Serial.print("\t");

    Serial.print("micros for cal:"); Serial.print(stopwatch);
    Serial.println();

    delay(1000);
}

