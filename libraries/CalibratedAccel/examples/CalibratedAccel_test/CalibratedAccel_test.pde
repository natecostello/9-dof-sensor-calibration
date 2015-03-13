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
float acc11 =  0.98286000; float acc21 = 0.0106020; float acc31 = -0.0057796;
float acc12 = -0.00626500; float acc22 = 0.9713500; float acc32 = -0.0030856;
float acc13 = -0.00051706; float acc23 = 0.0021296; float acc33 =  0.9804000;
float acc10 =  0.01515800; float acc20 = 0.0164400; float acc30 = -0.0677500;

// default address is 105
// specific I2C address may be passed here
LSM303DLHC accelMag;

float ax, ay, az;

float ax_uncal;
float ay_uncal;
float az_uncal;

CalibratedAccel myCalibratedAccel = CalibratedAccel(
    acc11, acc12, acc13, acc10
    acc21, acc22, acc23, acc20
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

    // set accel data rate to 200hz
    accelMag.setAccelOutputDataRate(200);
}

void loop()
{
    // read raw angular velocity measurements from device
    accelMag.getAcceleration(&ax, &ay, &az);

    // i think this is needed for the offset calibraition to work (unless called unscaled)
    ax = ax*0.0000625F;
    ay = ay*0.0000625F;
    az = az*0.0000625F;

    Serial.print("Raw Accelation:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");


    //calibrate:
    myCalibratedAccel.calibrateAccelerations(&ax, &ay, &az);

    Serial.print("Cal Accelation:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); 
    Serial.println();

    delay(1000);
}

