// LSM303DLCH_accel_cal.ino


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>

// I2Cdev and LSM303DLHC must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "LSM303DLHC.h"
#include "I2Cdev.h"

LSM303DLHC accelMag;

int16_t ax, ay, az;
int yx, yy, yz;


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

    // set scale to 2Gs.  Note this corresponds to a scaling factor of 0.0000625F.  Accuracy is to 4 decimals.
    accelMag.setAccelFullScale(2); 

    // set accel data rate to 200hz
    accelMag.setAccelOutputDataRate(200);

}

void loop() {
    Serial.println("DELETE BEFORE SAVING: Allow device to warm up for a few minutes before proceeding.");
    delay(1000);
    Serial.println("DELETE BEFORE SAVING: Align X-axis up, wait 10 seconds to allow stabilization, then send any key over serial to proceed");
    while (not Serial.available()){
        delay(10);
    }

    //format for datafile is Yx, Yy, Yz, Wx, Wy, Wz, 1
    //for this orientation Yx = 1, Yy = 0, Yz = 0
    yx = 1;
    yy = 0;
    yz = 0;

    for (int x = 0; x < 50; x++) {
        accelMag.getAcceleration(&ax, &ay, &az);
        Serial.print(yx); Serial.print(',');
        Serial.print(yy); Serial.print(',');
        Serial.print(yz); Serial.print(',');
        Serial.print(ax * 0.0000625F, 4); Serial.print(',');
        Serial.print(ay * 0.0000625F, 4); Serial.print(',');
        Serial.print(az * 0.0000625F, 4); Serial.println();
    }

    Serial.println("DELETE BEFORE SAVING: Align X-axis down, wait 10 seconds to allow stabilization, then send any key over serial to proceed");
    while (not Serial.available()){
        delay(10);
    }

    //format for datafile is Yx, Yy, Yz, Wx, Wy, Wz, 1
    //for this orientation Yx = -1, Yy = 0, Yz = 0
    yx = -1;
    yy = 0;
    yz = 0;

    for (int x = 0; x < 50; x++) {
        accelMag.getAcceleration(&ax, &ay, &az);
        Serial.print(yx); Serial.print(',');
        Serial.print(yy); Serial.print(',');
        Serial.print(yz); Serial.print(',');
        Serial.print(ax * 0.0000625F, 4); Serial.print(',');
        Serial.print(ay * 0.0000625F, 4); Serial.print(',');
        Serial.print(az * 0.0000625F, 4); Serial.println();
    }

    Serial.println("DELETE BEFORE SAVING: Align Y-axis up, wait 10 seconds to allow stabilization, then send any key over serial to proceed");
    while (not Serial.available()){
        delay(10);
    }

    //format for datafile is Yx, Yy, Yz, Wx, Wy, Wz, 1
    //for this orientation Yx = 0, Yy = 1, Yz = 0
    yx = 0;
    yy = 1;
    yz = 0;

    for (int x = 0; x < 50; x++) {
        accelMag.getAcceleration(&ax, &ay, &az);
        Serial.print(yx); Serial.print(',');
        Serial.print(yy); Serial.print(',');
        Serial.print(yz); Serial.print(',');
        Serial.print(ax * 0.0000625F, 4); Serial.print(',');
        Serial.print(ay * 0.0000625F, 4); Serial.print(',');
        Serial.print(az * 0.0000625F, 4); Serial.println();
    }

    Serial.println("DELETE BEFORE SAVING: Align Y-axis down, wait 10 seconds to allow stabilization, then send any key over serial to proceed");
    while (not Serial.available()){
        delay(10);
    }

    //format for datafile is Yx, Yy, Yz, Wx, Wy, Wz, 1
    //for this orientation Yx = 0, Yy = -1, Yz = 0
    yx = 0;
    yy = -1;
    yz = 0;

    for (int x = 0; x < 50; x++) {
        accelMag.getAcceleration(&ax, &ay, &az);
        Serial.print(yx); Serial.print(',');
        Serial.print(yy); Serial.print(',');
        Serial.print(yz); Serial.print(',');
        Serial.print(ax * 0.0000625F, 4); Serial.print(',');
        Serial.print(ay * 0.0000625F, 4); Serial.print(',');
        Serial.print(az * 0.0000625F, 4); Serial.println();
    }

    Serial.println("DELETE BEFORE SAVING: Align Z-axis up, wait 10 seconds to allow stabilization, then send any key over serial to proceed");
    while (not Serial.available()){
        delay(10);
    }

    //format for datafile is Yx, Yy, Yz, Wx, Wy, Wz, 1
    //for this orientation Yx = 0, Yy = 0, Yz = 1
    yx = 0;
    yy = 0;
    yz = 1;

    for (int x = 0; x < 50; x++) {
        accelMag.getAcceleration(&ax, &ay, &az);
        Serial.print(yx); Serial.print(',');
        Serial.print(yy); Serial.print(',');
        Serial.print(yz); Serial.print(',');
        Serial.print(ax * 0.0000625F, 4); Serial.print(',');
        Serial.print(ay * 0.0000625F, 4); Serial.print(',');
        Serial.print(az * 0.0000625F, 4); Serial.println();
    }
    
    Serial.println("DELETE BEFORE SAVING: Align Z-axis down, wait 10 seconds to allow stabilization, then send any key over serial to proceed");
    while (not Serial.available()){
        delay(10);
    }

    //format for datafile is Yx, Yy, Yz, Wx, Wy, Wz, 1
    //for this orientation Yx = 0, Yy = 0, Yz = -1
    yx = 0;
    yy = 0;
    yz = -1;

    for (int x = 0; x < 50; x++) {
        accelMag.getAcceleration(&ax, &ay, &az);
        Serial.print(yx); Serial.print(',');
        Serial.print(yy); Serial.print(',');
        Serial.print(yz); Serial.print(',');
        Serial.print(ax * 0.0000625F, 4); Serial.print(',');
        Serial.print(ay * 0.0000625F, 4); Serial.print(',');
        Serial.print(az * 0.0000625F, 4); Serial.println();
    }

    Serial.println("DELETE BEFORE SAVING:  Data Acquisition Complete.  Delete all lines marked as DELETE BEFORE SAVING.");
    Serial.println("DELETE BEFORE SAVING:  Save file as 'accelData.txt' and run octave/matlab file to complete computation of calibration constants.");
}

