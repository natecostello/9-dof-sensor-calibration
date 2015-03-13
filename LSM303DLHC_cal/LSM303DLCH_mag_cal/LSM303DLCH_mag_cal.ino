// LSM303DLCH_mag_cal.ino


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>

// I2Cdev and LSM303DLHC must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "LSM303DLHC.h"
#include "I2Cdev.h"

LSM303DLHC accelMag;

int16_t mx, my, mz;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    Serial.begin(9600);

    // initialize device
    Serial.println("DELETE BEFORE SAVING: Initializing I2C devices...");
    accelMag.initialize();

    // verify connection
    Serial.println("DELETE BEFORE SAVING: Testing device connections...");
    Serial.println(accelMag.testConnection() ? "DELETE BEFORE SAVING: LSM303DLHC connection successful" : "DELETE BEFORE SAVING: LSM303DLHC connection failed");

    // set accel data rate to 200hz
    accelMag.setAccelOutputDataRate(200);

    // set mag data rate to 220hz
    accelMag.setMagOutputDataRate(220);

    // set mag gain
    accelMag.setMagGain(1100);

    // enable mag
    accelMag.setMagMode(LSM303DLHC_MD_CONTINUOUS);



}

void loop() {
    Serial.println("DELETE BEFORE SAVING: Allow device to warm up for a few minutes before proceeding.");
    delay(1000);
    Serial.println("DELETE BEFORE SAVING: Send any key over serial, and then proceed to move device through all axes and orientations until data acquisition stops.");
    
    while (not Serial.available()){
        delay(10);
    }
    while (Serial.available()){
        Serial.read();
    }

    //format for datafile is mx, my, mz

    for (int x = 0; x < 1000; x++) {
        accelMag.getMag(&mx, &my, &mz);
        Serial.print(mx); Serial.print(',');
        Serial.print(my); Serial.print(',');
        Serial.print(mz); Serial.println();
        delay(100);
    }
    Serial.println("DELETE BEFORE SAVING:  Data Acquisition Complete.  Delete all lines marked as DELETE BEFORE SAVING.");
    Serial.println("DELETE BEFORE SAVING:  Save file as 'magData.txt' and run octave/matlab file to complete computation of calibration constants.");
    while (true){
        delay(1000);
    }
}

