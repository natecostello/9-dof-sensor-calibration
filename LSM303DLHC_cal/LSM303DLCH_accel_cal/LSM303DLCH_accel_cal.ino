// LSM303DLCH_accel_cal.ino


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>

// I2Cdev and LSM303DLHC must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "LSM303DLHC.h"
#include "I2Cdev.h"
#include <math.h>
#include <MatrixMath.h>

#define NUM_POSITIONS (12)

LSM303DLHC accelMag;

int16_t ax, ay, az;
float yx, yy, yz;
float wx, wy, wz;

float scale = 0.000061035F;


float Y_x_accumulated;
float Y_y_accumulated;
float Y_z_accumulated;
float W_x_accumulated;
float W_y_accumulated;
float W_z_accumulated;
float W_o_accumulated;

float Y_matrix[NUM_POSITIONS][3];
float W_matrix[NUM_POSITIONS][4];
float X_matrix[4][3];
float Wt_matrix[4][NUM_POSITIONS];
float WtW_matrix[4][4];
float WtW_inv_matrix[4][4];
float WtW_inv_Wt_matrix[4][NUM_POSITIONS];

float acc11 = 1;
float acc12 = 0;
float acc13 = 0;
float acc10 = 0;
float acc21 = 0;
float acc22 = 1;
float acc23 = 0;
float acc20 = 0;
float acc31 = 0;
float acc32 = 0;
float acc33 = 1;
float acc30 = 0;

int position;
int iteration = 200;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    Serial.begin(115200);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    accelMag.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(accelMag.testConnection() ? "LSM303DLHC connection successful" : "LSM303DLHC connection failed");

    // set scale to +-2Gs. Raw output for 1G = 1024 (16384 without 4-bit shift)
    accelMag.setAccelFullScale(2); 

    // set scale to +-4Gs. Raw output for 1G = 512.
    // accelMag.setAccelFullScale(4); 

    // high res output allows 12-bit vice 10-bit precision.
    accelMag.setAccelHighResOutputEnabled(true);

    // set accel data rate to 200hz
    accelMag.setAccelOutputDataRate(200);

}
void computeCalibrationConstants(){
    
    //DEBUG
    Serial.println("Y=");
    MatrixInst.Print((float*)Y_matrix, NUM_POSITIONS, 3);
    Serial.println("W=");
    MatrixInst.Print((float*)W_matrix, NUM_POSITIONS, 4);
    MatrixInst.Transpose((float*)W_matrix, NUM_POSITIONS, 4, (float*)Wt_matrix);
    Serial.println("W'=");
    MatrixInst.Print((float*)WtW_matrix, 4, NUM_POSITIONS);
    MatrixInst.Multiply((float*)Wt_matrix, (float*)W_matrix, 4, NUM_POSITIONS, 4, (float*)WtW_matrix);
    Serial.println("W'*W=");
    MatrixInst.Print((float*)WtW_matrix, 4, 4);
    MatrixInst.Invert((float*)WtW_matrix, 4);
    Serial.println("(W'*W)^(-1)=");
    MatrixInst.Print((float*)WtW_matrix, 4, 4);
    MatrixInst.Multiply((float*)WtW_matrix, (float*)Wt_matrix, 4, 4, NUM_POSITIONS, (float*)WtW_inv_Wt_matrix);
    Serial.println("(W'*W)^(-1) * W'=");
    MatrixInst.Print((float*)WtW_inv_Wt_matrix, 4, NUM_POSITIONS);
    MatrixInst.Multiply((float*)WtW_inv_Wt_matrix, (float*)Y_matrix, 4, NUM_POSITIONS, 3, (float*)X_matrix);
    Serial.println("(W'*W)^(-1) * W' * Y=X= ");
    MatrixInst.Print((float*)X_matrix, 4, 3);
    //\DEBUG
   
    // FML: Zero indexing AHHHHHH
    acc11 = X_matrix[0][0];
    acc12 = X_matrix[1][0];
    acc13 = X_matrix[2][0];
    acc10 = X_matrix[3][0];
    acc21 = X_matrix[0][1];
    acc22 = X_matrix[1][1];
    acc23 = X_matrix[2][1];
    acc20 = X_matrix[3][1];
    acc31 = X_matrix[0][2];
    acc32 = X_matrix[1][2];
    acc33 = X_matrix[2][2];
    acc30 = X_matrix[3][2];

    Serial.print("float acc11 = "); Serial.print(acc11,14); Serial.print(";\n");
    Serial.print("float acc21 = "); Serial.print(acc21,14); Serial.print(";\n");
    Serial.print("float acc31 = "); Serial.print(acc31,14); Serial.print(";\n");
    Serial.print("float acc12 = "); Serial.print(acc12,14); Serial.print(";\n");
    Serial.print("float acc22 = "); Serial.print(acc22,14); Serial.print(";\n");
    Serial.print("float acc32 = "); Serial.print(acc32,14); Serial.print(";\n");
    Serial.print("float acc13 = "); Serial.print(acc13,14); Serial.print(";\n");
    Serial.print("float acc23 = "); Serial.print(acc23,14); Serial.print(";\n");
    Serial.print("float acc33 = "); Serial.print(acc33,14); Serial.print(";\n");
    Serial.print("float acc10 = "); Serial.print(acc10,14); Serial.print(";\n");
    Serial.print("float acc20 = "); Serial.print(acc20,14); Serial.print(";\n");
    Serial.print("float acc30 = "); Serial.print(acc30,14); Serial.print(";\n");
    
}


void loop() {
    Serial.println(F("Allow device to warm up for a few minutes before proceeding."));
    delay(1000);
    
    // position 0
    position = 0;
    Serial.println(F("Align X-axis up, wait 10 seconds to allow stabilization, then send any key over serial to proceed"));
    while (not Serial.available()){
        delay(10);
    }
    while (Serial.available()){
        Serial.read();
    }
    
    //for this orientation Yx = 1.0, Yy = 0, Yz = 0
    yx = 1.0;
    yy = 0.0;
    yz = 0.0;
    Y_x_accumulated = 0;
    Y_y_accumulated = 0;
    Y_z_accumulated = 0;
    W_x_accumulated = 0;
    W_y_accumulated = 0;
    W_z_accumulated = 0;
    W_o_accumulated = 0;

    for (int i = 0; i < iteration; i++){

        accelMag.getAcceleration(&ax, &ay, &az);
        wx = scale*ax;
        wy = scale*ay;
        wz = scale*az;

        // Serial.println("Scaling Completed");

        Y_x_accumulated += yx;
        Y_y_accumulated += yy;
        Y_z_accumulated += yz;
        W_x_accumulated += wx;
        W_y_accumulated += wy;
        W_z_accumulated += wz;
        W_o_accumulated += 1;

        // Serial.println("Accumulation Completed - about to delay");

        // wait for accel data to refresh
        delay(5);
    }

    Y_matrix[position][0] = Y_x_accumulated;
    Y_matrix[position][1] = Y_y_accumulated;
    Y_matrix[position][2] = Y_z_accumulated;
    W_matrix[position][0] = W_x_accumulated;
    W_matrix[position][1] = W_y_accumulated;
    W_matrix[position][2] = W_z_accumulated;
    W_matrix[position][3] = W_o_accumulated;

    // position 1
    position = 1;
    Serial.println(F("With X-axis up, rotate 180 degrees, wait 10 seconds to allow stabilization, then send any key over serial to proceed"));
    while (not Serial.available()){
        delay(10);
    }
    while (Serial.available()){
        Serial.read();
    }
    
    // for this orientation Yx = 1.0, Yy = 0, Yz = 0
    yx = 1.0;
    yy = 0.0;
    yz = 0.0;
    Y_x_accumulated = 0;
    Y_y_accumulated = 0;
    Y_z_accumulated = 0;
    W_x_accumulated = 0;
    W_y_accumulated = 0;
    W_z_accumulated = 0;
    W_o_accumulated = 0;

    for (int i = 0; i < iteration; i++){
        accelMag.getAcceleration(&ax, &ay, &az);
        wx = scale*ax;
        wy = scale*ay;
        wz = scale*az;
        Y_x_accumulated += yx;
        Y_y_accumulated += yy;
        Y_z_accumulated += yz;
        W_x_accumulated += wx;
        W_y_accumulated += wy;
        W_z_accumulated += wz;
        W_o_accumulated += 1;
        delay(5);
    }

    Y_matrix[position][0] = Y_x_accumulated;
    Y_matrix[position][1] = Y_y_accumulated;
    Y_matrix[position][2] = Y_z_accumulated;
    W_matrix[position][0] = W_x_accumulated;
    W_matrix[position][1] = W_y_accumulated;
    W_matrix[position][2] = W_z_accumulated;
    W_matrix[position][3] = W_o_accumulated;

    // position 2
    position = 2;
    Serial.println(F("Align X-axis down, wait 10 seconds to allow stabilization, then send any key over serial to proceed"));
    while (not Serial.available()){
        delay(10);
    }
    while (Serial.available()){
        Serial.read();
    }

    //for this orientation Yx = -1.0, Yy = 0, Yz = 0
    yx = -1.0;
    yy = 0.0;
    yz = 0.0;
    Y_x_accumulated = 0;
    Y_y_accumulated = 0;
    Y_z_accumulated = 0;
    W_x_accumulated = 0;
    W_y_accumulated = 0;
    W_z_accumulated = 0;
    W_o_accumulated = 0;

    for (int i = 0; i < iteration; i++){

        accelMag.getAcceleration(&ax, &ay, &az);
        wx = scale*ax;
        wy = scale*ay;
        wz = scale*az;
        Y_x_accumulated += yx;
        Y_y_accumulated += yy;
        Y_z_accumulated += yz;
        W_x_accumulated += wx;
        W_y_accumulated += wy;
        W_z_accumulated += wz;
        W_o_accumulated += 1;
        delay(5);
    }

    Y_matrix[position][0] = Y_x_accumulated;
    Y_matrix[position][1] = Y_y_accumulated;
    Y_matrix[position][2] = Y_z_accumulated;
    W_matrix[position][0] = W_x_accumulated;
    W_matrix[position][1] = W_y_accumulated;
    W_matrix[position][2] = W_z_accumulated;
    W_matrix[position][3] = W_o_accumulated;

    // position 3
    position = 3;
    Serial.println(F("With X-axis down, rotate 180 degrees, wait 10 seconds to allow stabilization, then send any key over serial to proceed"));
    while (not Serial.available()){
        delay(10);
    }
    while (Serial.available()){
        Serial.read();
    }

    //for this orientation Yx = -1.0, Yy = 0, Yz = 0
    yx = -1.0;
    yy = 0.0;
    yz = 0.0;
    Y_x_accumulated = 0;
    Y_y_accumulated = 0;
    Y_z_accumulated = 0;
    W_x_accumulated = 0;
    W_y_accumulated = 0;
    W_z_accumulated = 0;
    W_o_accumulated = 0;

    for (int i = 0; i < iteration; i++){

        accelMag.getAcceleration(&ax, &ay, &az);
        wx = scale*ax;
        wy = scale*ay;
        wz = scale*az;
        Y_x_accumulated += yx;
        Y_y_accumulated += yy;
        Y_z_accumulated += yz;
        W_x_accumulated += wx;
        W_y_accumulated += wy;
        W_z_accumulated += wz;
        W_o_accumulated += 1;
        delay(5);
    }

    Y_matrix[position][0] = Y_x_accumulated;
    Y_matrix[position][1] = Y_y_accumulated;
    Y_matrix[position][2] = Y_z_accumulated;
    W_matrix[position][0] = W_x_accumulated;
    W_matrix[position][1] = W_y_accumulated;
    W_matrix[position][2] = W_z_accumulated;
    W_matrix[position][3] = W_o_accumulated;

    // position 4
    position = 4;
    Serial.println(F("Align Y-axis up, wait 10 seconds to allow stabilization, then send any key over serial to proceed"));
    while (not Serial.available()){
        delay(10);
    }
    while (Serial.available()){
        Serial.read();
    }

    //for this orientation Yx = 0.0, Yy = 1.0, Yz = 0
    yx = 0.0;
    yy = 1.0;
    yz = 0.0;
    Y_x_accumulated = 0;
    Y_y_accumulated = 0;
    Y_z_accumulated = 0;
    W_x_accumulated = 0;
    W_y_accumulated = 0;
    W_z_accumulated = 0;
    W_o_accumulated = 0;

    for (int i = 0; i < iteration; i++){

        accelMag.getAcceleration(&ax, &ay, &az);
        wx = scale*ax;
        wy = scale*ay;
        wz = scale*az;
        Y_x_accumulated += yx;
        Y_y_accumulated += yy;
        Y_z_accumulated += yz;
        W_x_accumulated += wx;
        W_y_accumulated += wy;
        W_z_accumulated += wz;
        W_o_accumulated += 1;
        delay(5);
    }

    Y_matrix[position][0] = Y_x_accumulated;
    Y_matrix[position][1] = Y_y_accumulated;
    Y_matrix[position][2] = Y_z_accumulated;
    W_matrix[position][0] = W_x_accumulated;
    W_matrix[position][1] = W_y_accumulated;
    W_matrix[position][2] = W_z_accumulated;
    W_matrix[position][3] = W_o_accumulated;

    // position 5
    position = 5;
    Serial.println(F("With Y-axis up, rotate 180 degrees, wait 10 seconds to allow stabilization, then send any key over serial to proceed"));
    while (not Serial.available()){
        delay(10);
    }
    while (Serial.available()){
        Serial.read();
    }

    yx = 0.0;
    yy = 1.0;
    yz = 0.0;
    Y_x_accumulated = 0;
    Y_y_accumulated = 0;
    Y_z_accumulated = 0;
    W_x_accumulated = 0;
    W_y_accumulated = 0;
    W_z_accumulated = 0;
    W_o_accumulated = 0;

    for (int i = 0; i < iteration; i++){

        accelMag.getAcceleration(&ax, &ay, &az);
        wx = scale*ax;
        wy = scale*ay;
        wz = scale*az;
        Y_x_accumulated += yx;
        Y_y_accumulated += yy;
        Y_z_accumulated += yz;
        W_x_accumulated += wx;
        W_y_accumulated += wy;
        W_z_accumulated += wz;
        W_o_accumulated += 1;
        delay(5);
    }

    Y_matrix[position][0] = Y_x_accumulated;
    Y_matrix[position][1] = Y_y_accumulated;
    Y_matrix[position][2] = Y_z_accumulated;
    W_matrix[position][0] = W_x_accumulated;
    W_matrix[position][1] = W_y_accumulated;
    W_matrix[position][2] = W_z_accumulated;
    W_matrix[position][3] = W_o_accumulated;

    // position 6
    position = 6;
    Serial.println(F("Align Y-axis down, wait 10 seconds to allow stabilization, then send any key over serial to proceed"));
    while (not Serial.available()){
        delay(10);
    }
    while (Serial.available()){
        Serial.read();
    }

    //for this orientation Yx = 0.0, Yy = -1.0, Yz = 0
    yx = 0.0;
    yy = -1.0;
    yz = 0.0;
    Y_x_accumulated = 0;
    Y_y_accumulated = 0;
    Y_z_accumulated = 0;
    W_x_accumulated = 0;
    W_y_accumulated = 0;
    W_z_accumulated = 0;
    W_o_accumulated = 0;

    for (int i = 0; i < iteration; i++){

        accelMag.getAcceleration(&ax, &ay, &az);
        wx = scale*ax;
        wy = scale*ay;
        wz = scale*az;
        Y_x_accumulated += yx;
        Y_y_accumulated += yy;
        Y_z_accumulated += yz;
        W_x_accumulated += wx;
        W_y_accumulated += wy;
        W_z_accumulated += wz;
        W_o_accumulated += 1;
        delay(5);
    }

    Y_matrix[position][0] = Y_x_accumulated;
    Y_matrix[position][1] = Y_y_accumulated;
    Y_matrix[position][2] = Y_z_accumulated;
    W_matrix[position][0] = W_x_accumulated;
    W_matrix[position][1] = W_y_accumulated;
    W_matrix[position][2] = W_z_accumulated;
    W_matrix[position][3] = W_o_accumulated;

    // position 7
    position = 7;
    Serial.println(F("With Y-axis down, rotate 180 degrees, wait 10 seconds to allow stabilization, then send any key over serial to proceed"));
    while (not Serial.available()){
        delay(10);
    }
    while (Serial.available()){
        Serial.read();
    }

    //for this orientation Yx = 0.0, Yy = -1.0, Yz = 0
    yx = 0.0;
    yy = -1.0;
    yz = 0.0;
    Y_x_accumulated = 0;
    Y_y_accumulated = 0;
    Y_z_accumulated = 0;
    W_x_accumulated = 0;
    W_y_accumulated = 0;
    W_z_accumulated = 0;
    W_o_accumulated = 0;

    for (int i = 0; i < iteration; i++){

        accelMag.getAcceleration(&ax, &ay, &az);
        wx = scale*ax;
        wy = scale*ay;
        wz = scale*az;
        Y_x_accumulated += yx;
        Y_y_accumulated += yy;
        Y_z_accumulated += yz;
        W_x_accumulated += wx;
        W_y_accumulated += wy;
        W_z_accumulated += wz;
        W_o_accumulated += 1;
        delay(5);
    }

    Y_matrix[position][0] = Y_x_accumulated;
    Y_matrix[position][1] = Y_y_accumulated;
    Y_matrix[position][2] = Y_z_accumulated;
    W_matrix[position][0] = W_x_accumulated;
    W_matrix[position][1] = W_y_accumulated;
    W_matrix[position][2] = W_z_accumulated;
    W_matrix[position][3] = W_o_accumulated;

    // position 8
    position = 8;
    Serial.println(F("Align Z-axis up, wait 10 seconds to allow stabilization, then send any key over serial to proceed"));
    while (not Serial.available()){
        delay(10);
    }
    while (Serial.available()){
        Serial.read();
    }

    //for this orientation Yx = 0.0, Yy = 0, Yz = 1.0
    yx = 0.0;
    yy = 0.0;
    yz = 1.0;
    Y_x_accumulated = 0;
    Y_y_accumulated = 0;
    Y_z_accumulated = 0;
    W_x_accumulated = 0;
    W_y_accumulated = 0;
    W_z_accumulated = 0;
    W_o_accumulated = 0;

    for (int i = 0; i < iteration; i++){

        accelMag.getAcceleration(&ax, &ay, &az);
        wx = scale*ax;
        wy = scale*ay;
        wz = scale*az;
        Y_x_accumulated += yx;
        Y_y_accumulated += yy;
        Y_z_accumulated += yz;
        W_x_accumulated += wx;
        W_y_accumulated += wy;
        W_z_accumulated += wz;
        W_o_accumulated += 1;
        delay(5);
    }

    Y_matrix[position][0] = Y_x_accumulated;
    Y_matrix[position][1] = Y_y_accumulated;
    Y_matrix[position][2] = Y_z_accumulated;
    W_matrix[position][0] = W_x_accumulated;
    W_matrix[position][1] = W_y_accumulated;
    W_matrix[position][2] = W_z_accumulated;
    W_matrix[position][3] = W_o_accumulated;

    // position 9
    position = 9;
    Serial.println(F("With Z-axis up, rotate 180 degrees, wait 10 seconds to allow stabilization, then send any key over serial to proceed"));
    while (not Serial.available()){
        delay(10);
    }
    while (Serial.available()){
        Serial.read();
    }

    //for this orientation Yx = 0.0, Yy = 0, Yz = 1.0
    yx = 0.0;
    yy = 0.0;
    yz = 1.0;
    Y_x_accumulated = 0;
    Y_y_accumulated = 0;
    Y_z_accumulated = 0;
    W_x_accumulated = 0;
    W_y_accumulated = 0;
    W_z_accumulated = 0;
    W_o_accumulated = 0;

    for (int i = 0; i < iteration; i++){

        accelMag.getAcceleration(&ax, &ay, &az);
        wx = scale*ax;
        wy = scale*ay;
        wz = scale*az;

        Y_x_accumulated += yx;
        Y_y_accumulated += yy;
        Y_z_accumulated += yz;
        W_x_accumulated += wx;
        W_y_accumulated += wy;
        W_z_accumulated += wz;
        W_o_accumulated += 1;

        // wait for accel data to refresh
        delay(5);
    }

    Y_matrix[position][0] = Y_x_accumulated;
    Y_matrix[position][1] = Y_y_accumulated;
    Y_matrix[position][2] = Y_z_accumulated;
    W_matrix[position][0] = W_x_accumulated;
    W_matrix[position][1] = W_y_accumulated;
    W_matrix[position][2] = W_z_accumulated;
    W_matrix[position][3] = W_o_accumulated;

    // position 10
    position = 10;
    Serial.println(F("Align Z-axis down, wait 10 seconds to allow stabilization, then send any key over serial to proceed"));
    while (not Serial.available()){
        delay(10);
    }
    while (Serial.available()){
        Serial.read();
    }


    // for this orientation Yx = 0.0, Yy = 0, Yz = -1.0
    yx = 0.0;
    yy = 0.0;
    yz = -1.0;
    Y_x_accumulated = 0;
    Y_y_accumulated = 0;
    Y_z_accumulated = 0;
    W_x_accumulated = 0;
    W_y_accumulated = 0;
    W_z_accumulated = 0;
    W_o_accumulated = 0;

    for (int i = 0; i < iteration; i++){

        accelMag.getAcceleration(&ax, &ay, &az);
        wx = scale*ax;
        wy = scale*ay;
        wz = scale*az;

        Y_x_accumulated += yx;
        Y_y_accumulated += yy;
        Y_z_accumulated += yz;
        W_x_accumulated += wx;
        W_y_accumulated += wy;
        W_z_accumulated += wz;
        W_o_accumulated += 1;
        delay(5);
    }

    Y_matrix[position][0] = Y_x_accumulated;
    Y_matrix[position][1] = Y_y_accumulated;
    Y_matrix[position][2] = Y_z_accumulated;
    W_matrix[position][0] = W_x_accumulated;
    W_matrix[position][1] = W_y_accumulated;
    W_matrix[position][2] = W_z_accumulated;
    W_matrix[position][3] = W_o_accumulated;

    // position 11
    position = 11;
    Serial.println(F("With Z-axis down, rotate 180 degrees, wait 10 seconds to allow stabilization, then send any key over serial to proceed"));
    while (not Serial.available()){
        delay(10);
    }
    while (Serial.available()){
        Serial.read();
    }

    // for this orientation Yx = 0.0, Yy = 0, Yz = -1.0
    yx = 0.0;
    yy = 0.0;
    yz = -1.0;
    Y_x_accumulated = 0;
    Y_y_accumulated = 0;
    Y_z_accumulated = 0;
    W_x_accumulated = 0;
    W_y_accumulated = 0;
    W_z_accumulated = 0;
    W_o_accumulated = 0;

    for (int i = 0; i < iteration; i++){

        accelMag.getAcceleration(&ax, &ay, &az);
        wx = scale*ax;
        wy = scale*ay;
        wz = scale*az;

        Y_x_accumulated += yx;
        Y_y_accumulated += yy;
        Y_z_accumulated += yz;
        W_x_accumulated += wx;
        W_y_accumulated += wy;
        W_z_accumulated += wz;
        W_o_accumulated += 1;

        delay(5);
    }

    Y_matrix[position][0] = Y_x_accumulated;
    Y_matrix[position][1] = Y_y_accumulated;
    Y_matrix[position][2] = Y_z_accumulated;
    W_matrix[position][0] = W_x_accumulated;
    W_matrix[position][1] = W_y_accumulated;
    W_matrix[position][2] = W_z_accumulated;
    W_matrix[position][3] = W_o_accumulated;

    // crunch coefficients
    computeCalibrationConstants();

    while (true){
        delay(1000);
    }
}

