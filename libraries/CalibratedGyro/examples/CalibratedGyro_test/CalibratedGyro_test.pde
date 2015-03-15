// This sketch tests the CalibratedGyro library
// by Nate Costello <email>
// Created X March 2015

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>

// I2Cdev and L3GD20H must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <L3GD20H.h>

// This is the library to be testing
#include <CalibratedGyro.h>

// Calibration constants from AccelCal.m
float gcc11 = 0.996335;
float gcc12 = 0.000000;
float gcc13 = 0.000000;
float gcc10 = 148.316109;
float gcc21 = 0.000000;
float gcc22 = 1.027129;
float gcc23 = 0.000000;
float gcc20 = -87.516717;
float gcc31 = 0.000000;
float gcc32 = 0.000000;
float gcc33 = 1.001950;
float gcc30 = -91.682371;

// LSB scaling to dps for +-500 dps range setting
float scale = 0.0175;

// stopwatch to time calibration
uint16_t stopwatch;

L3GD20H gyro;

int16_t rx, ry, rz;

CalibratedGyro myCalibratedGyro(
    gcc11, gcc12, gcc13, gcc10,
    gcc21, gcc22, gcc23, gcc20,
    gcc31, gcc32, gcc33, gcc30);

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    Serial.begin(115200);

    // initialize device
    Serial.println("Initializing I2C devices...");
    gyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(gyro.testConnection() ? "L3GD20H connection successful" : "L3GD20H connection failed");

    // set scale to 500dps
    gyro.setFullScale(500);

    // set gyro data rate to 200hz
    gyro.setOutputDataRate(200);
}

void loop()
{
    // read raw angular velocity measurements from device
    gyro.getAngularVelocity(&rx, &ry, &rz);

    Serial.print("Raw AngularV:\t");
    Serial.print(rx * scale); Serial.print("\t");
    Serial.print(ry * scale); Serial.print("\t");
    Serial.print(rz * scale); Serial.print("\t");


    //calibrate and time calibration
    stopwatch = micros();
    myCalibratedGyro.calibrateAngularVelocities(&rx, &ry, &rz);
    stopwatch = micros() - stopwatch;

    Serial.print("Cal AngularV:\t");
    Serial.print(rx * scale); Serial.print("\t");
    Serial.print(ry * scale); Serial.print("\t");
    Serial.print(rz * scale); Serial.print("\t");

    Serial.print("micros for cal:"); Serial.print(stopwatch);
    Serial.println();

    delay(100);
}

