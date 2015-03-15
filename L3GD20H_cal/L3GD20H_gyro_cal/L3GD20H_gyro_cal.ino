// L3GD20H_gyro_cal.ino

// For methodoly used to calibrate the gyros refer to:
// "A Simple Calibration For Mems Gyroscopes" by Mark Looney of Analog Devices

// Timing determinism:
// A gyro read followed by Serial printing takes about 1.5 milliseconds to run at 115200 baud.
// The gyro read takes about 1 milliseconds of that time.
// Overall cycle timing is accurate to about 8 microseconds on a 10 millisecond period.


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>

// I2Cdev and L3GD20H must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "L3GD20H.h"
#include "I2Cdev.h"
#include "TimerOne.h"

L3GD20H gyro;

volatile bool read_now = false;

int16_t rx, ry, rz;

void setup() {
    Timer1.initialize(10000); //timer will tick every 0.01 seconds
    Timer1.attachInterrupt(updateReadNow);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    Serial.begin(115200);

    // initialize device
    Serial.println(F("DELETE BEFORE SAVING: Initializing I2C devices..."));
    gyro.initialize();

    // verify connection
    Serial.println(F("DELETE BEFORE SAVING: Testing device connections..."));
    Serial.println(gyro.testConnection() ? "DELETE BEFORE SAVING: L3GD20H connection successful" : "DELETE BEFORE SAVING: L3GD20H connection failed");

    // set scale to +-500dps. scaling factor should be 0.0175 per LSB
    gyro.setFullScale(500); 

    // set scale to +-2000dps.  scaling factor should be 0.07 per LSB
    // gyro.setAccelFullScale(4); 

    // set accel data rate to 100hz (for memory)
    gyro.setOutputDataRate(100);

    delay(1000);

}

void updateReadNow(void){
    read_now = true;
}

void loop() {

    Serial.println(F("DELETE BEFORE SAVING: Allow device to warm up for a few minutes before proceeding."));
    delay(1000);
    Serial.println(F("DELETE BEFORE SAVING: Align X-axis up, wait 10 seconds to allow stabilization, then send any key over serial to proceed and another to stop"));
    while (not Serial.available()){
        delay(10);
    }

    //clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    // write octave text format x y z 
    Serial.println("# name: x_u_static");
    Serial.println("# type: matrix");
    Serial.print("# rows: "); Serial.println("INSERT NUMBER OF ROWS");
    Serial.println("# columns: 3");

    //clear the read now flag
    noInterrupts();
    read_now = false;
    interrupts();

    // listen for a key sent to stop gather data until then
    while (not Serial.available()){
        if (read_now){
            gyro.getAngularVelocity(&rx, &ry, &rz);
            Serial.print(rx); Serial.print(" ");
            Serial.print(ry); Serial.print(" ");
            Serial.print(rz); Serial.println();
            read_now = false;
        }
    }

    // clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    Serial.println(F("DELETE BEFORE SAVING: With X-axis up, send key, rotate 180 degrees ClockWise, and send key."));
    while (not Serial.available()){
        delay(10);
    }

    // clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    // write octave text format x y z 
    Serial.println("# name: x_u_cw");
    Serial.println("# type: matrix");
    Serial.print("# rows: "); Serial.println("INSERT NUMBER OF ROWS");
    Serial.println("# columns: 3");

    //clear the read now flag
    noInterrupts();
    read_now = false;
    interrupts();

    // listen for a key sent to stop, gather data until then
    while (not Serial.available()){
        if (read_now){
            gyro.getAngularVelocity(&rx, &ry, &rz);
            Serial.print(rx); Serial.print(" ");
            Serial.print(ry); Serial.print(" ");
            Serial.print(rz); Serial.println();
            read_now = false;
        }
    }

    // clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    Serial.println(F("DELETE BEFORE SAVING: With X-axis up, send key, rotate 180 degrees CounterClockWise, and send key."));
    while (not Serial.available()){
        delay(10);
    }

    // clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    // write octave text format x y z 
    Serial.println("# name: x_u_ccw");
    Serial.println("# type: matrix");
    Serial.print("# rows: "); Serial.println("INSERT NUMBER OF ROWS");
    Serial.println("# columns: 3");

    //clear the read now flag
    noInterrupts();
    read_now = false;
    interrupts();

    // listen for a key sent to stop, gather data until then
    while (not Serial.available()){
        if (read_now){
            gyro.getAngularVelocity(&rx, &ry, &rz);
            Serial.print(rx); Serial.print(" ");
            Serial.print(ry); Serial.print(" ");
            Serial.print(rz); Serial.println();
            read_now = false;
        }
    }

    // clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    //Y Axis ////////////////////////////////////////////////////////////////////////////////////////

    Serial.println(F("DELETE BEFORE SAVING: Align Y-axis up, wait 10 seconds to allow stabilization, then send any key over serial to proceed and another to stop"));
    while (not Serial.available()){
        delay(10);
    }

    //clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    // write octave text format x y z 
    Serial.println("# name: y_u_static");
    Serial.println("# type: matrix");
    Serial.print("# rows: "); Serial.println("INSERT NUMBER OF ROWS");
    Serial.println("# columns: 3");

    //clear the read now flag
    noInterrupts();
    read_now = false;
    interrupts();

    // listen for a key sent to stop gather data until then
    while (not Serial.available()){
        if (read_now){
            gyro.getAngularVelocity(&rx, &ry, &rz);
            Serial.print(rx); Serial.print(" ");
            Serial.print(ry); Serial.print(" ");
            Serial.print(rz); Serial.println();
            read_now = false;
        }
    }

    // clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    Serial.println(F("DELETE BEFORE SAVING: With Y-axis up, send key, rotate 180 degrees ClockWise, and send key."));
    while (not Serial.available()){
        delay(10);
    }

    // clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    // write octave text format x y z 
    Serial.println("# name: y_u_cw");
    Serial.println("# type: matrix");
    Serial.print("# rows: "); Serial.println("INSERT NUMBER OF ROWS");
    Serial.println("# columns: 3");

    //clear the read now flag
    noInterrupts();
    read_now = false;
    interrupts();

    // listen for a key sent to stop, gather data until then
    while (not Serial.available()){
        if (read_now){
            gyro.getAngularVelocity(&rx, &ry, &rz);
            Serial.print(rx); Serial.print(" ");
            Serial.print(ry); Serial.print(" ");
            Serial.print(rz); Serial.println();
            read_now = false;
        }
    }

    // clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    Serial.println(F("DELETE BEFORE SAVING: With Y-axis up, send key, rotate 180 degrees CounterClockWise, and send key."));
    while (not Serial.available()){
        delay(10);
    }

    // clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    // write octave text format x y z 
    Serial.println("# name: y_u_ccw");
    Serial.println("# type: matrix");
    Serial.print("# rows: "); Serial.println("INSERT NUMBER OF ROWS");
    Serial.println("# columns: 3");

    //clear the read now flag
    noInterrupts();
    read_now = false;
    interrupts();

    // listen for a key sent to stop, gather data until then
    while (not Serial.available()){
        if (read_now){
            gyro.getAngularVelocity(&rx, &ry, &rz);
            Serial.print(rx); Serial.print(" ");
            Serial.print(ry); Serial.print(" ");
            Serial.print(rz); Serial.println();
            read_now = false;
        }
    }

    // clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    //Z Axis
    Serial.println(F("DELETE BEFORE SAVING: Align Z-axis up, wait 10 seconds to allow stabilization, then send any key over serial to proceed and another to stop"));
    while (not Serial.available()){
        delay(10);
    }

    //clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    // write octave text format x y z 
    Serial.println("# name: z_u_static");
    Serial.println("# type: matrix");
    Serial.print("# rows: "); Serial.println("INSERT NUMBER OF ROWS");
    Serial.println("# columns: 3");

    //clear the read now flag
    noInterrupts();
    read_now = false;
    interrupts();

    // listen for a key sent to stop gather data until then
    while (not Serial.available()){
        if (read_now){
            gyro.getAngularVelocity(&rx, &ry, &rz);
            Serial.print(rx); Serial.print(" ");
            Serial.print(ry); Serial.print(" ");
            Serial.print(rz); Serial.println();
            read_now = false;
        }
    }

    // clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    Serial.println(F("DELETE BEFORE SAVING: With Z-axis up, send key, rotate 180 degrees ClockWise, and send key."));
    while (not Serial.available()){
        delay(10);
    }

    // clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    // write octave text format x y z 
    Serial.println("# name: z_u_cw");
    Serial.println("# type: matrix");
    Serial.print("# rows: "); Serial.println("INSERT NUMBER OF ROWS");
    Serial.println("# columns: 3");

    //clear the read now flag
    noInterrupts();
    read_now = false;
    interrupts();

    // listen for a key sent to stop, gather data until then
    while (not Serial.available()){
        if (read_now){
            gyro.getAngularVelocity(&rx, &ry, &rz);
            Serial.print(rx); Serial.print(" ");
            Serial.print(ry); Serial.print(" ");
            Serial.print(rz); Serial.println();
            read_now = false;
        }
    }

    // clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    Serial.println(F("DELETE BEFORE SAVING: With Z-axis up, send key, rotate 180 degrees CounterClockWise, and send key."));
    while (not Serial.available()){
        delay(10);
    }

    // clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    // write octave text format x y z 
    Serial.println("# name: z_u_ccw");
    Serial.println("# type: matrix");
    Serial.print("# rows: "); Serial.println("INSERT NUMBER OF ROWS");
    Serial.println("# columns: 3");

    //clear the read now flag
    noInterrupts();
    read_now = false;
    interrupts();

    // listen for a key sent to stop, gather data until then
    while (not Serial.available()){
        if (read_now){
            gyro.getAngularVelocity(&rx, &ry, &rz);
            Serial.print(rx); Serial.print(" ");
            Serial.print(ry); Serial.print(" ");
            Serial.print(rz); Serial.println();
            read_now = false;
        }
    }

    // clear serial buffer
    while (Serial.available()){
        Serial.read();
    }

    // write octave text format x y z 
    Serial.println("# name: delta_t");
    Serial.println("# type: scalar");
    Serial.println(0.01);
    Serial.println(F("DELETE BEFORE SAVING: Callibration data acquired.  Clean file and save as 'gyroData.txt'.  Then run 'gyroCal.m'."));

    while (true){
        delay(1000);
    }
}
