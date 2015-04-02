// LSM303DLHC_accel_cal_online.pde

// This sketch will perform online generation of calibration constants for a LSM303DLHC.  

#include "LSM303DLHC.h"
#include "CalibratedAccel.h"
#include "CalibratedGyro.h"
#include "CalibratedMag.h"
#include "I2Cdev.h"
// #include "config/known_16bit_timers.h"
// #include "TimerOne.h"
#include <DueTimer.h>
//#include "utility/twi.h"
#include "Wire.h"
extern "C" {
    // #include "MadgwickAHRS.h"
    #include "MahonyAHRS.h"
};
#include <math.h>
#include "Quaternion.h"
#include <MatrixMath.h>

#define NUM_POSITIONS (6)

LSM303DLHC accelMag;

int16_t ax, ay, az;
float axf, ayf, azf;
int16_t yx, yy, yz;
float q0_last = 1;
float q1_last = 1;
float q2_last = 1;
float q3_last = 1;
float epsilon = 0.000001;

float gs_x;
float gs_y;
float gs_z;

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


void setup(){
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    Serial.begin(115200);

    // initialize device
    Serial.println(F("DELETE BEFORE SAVING: Initializing I2C devices..."));
    accelMag.initialize();

    // verify connection
    Serial.println(F("DELETE BEFORE SAVING: Testing device connections..."));
    Serial.println(accelMag.testConnection() ? "DELETE BEFORE SAVING: LSM303DLHC connection successful" : "DELETE BEFORE SAVING: LSM303DLHC connection failed");

    // set scale to +-2Gs. Raw output for 1G = 1024 (16384 without 4-bit shift)
    accelMag.setAccelFullScale(2); 

    // set scale to +-4Gs. Raw output for 1G = 512.
    // accelMag.setAccelFullScale(4); 

    // high res output allows 12-bit vice 10-bit precision.
    accelMag.setAccelHighResOutputEnabled(true);

    // set accel data rate to 200hz
    accelMag.setAccelOutputDataRate(100);

    // initilize pin 13 for blinking
    pinMode(13, OUTPUT);
}

bool stableQuaternion(){
    if ((abs(q0_last - q0) < epsilon) and
        (abs(q1_last - q1) < epsilon) and
        (abs(q2_last - q2) < epsilon) and
        (abs(q3_last - q3) < epsilon)){
        Serial.println("Quaternion is Stable");
        return true;
    } else {
        Serial.print("q0e ");  Serial.print(abs(q0_last - q0),14);
        Serial.print(" q1e "); Serial.print(abs(q1_last - q1),14);
        Serial.print(" q2e "); Serial.print(abs(q2_last - q2),14);
        Serial.print(" q3e "); Serial.print(abs(q3_last - q3),14);
        Serial.println();
        return false;
    }
}

void getPositionData(int position){
    q0_last = 1;
    q1_last = 1;
    q2_last = 1;
    q3_last = 1;

    Serial.println("Inside getPositionData");
        // wait for stable quaternion
    while (not stableQuaternion()){
        accelMag.getAcceleration(&ax, &ay, &az);
        axf = -0.000061035F*ax;
        ayf = -0.000061035F*ay;
        azf = -0.000061035F*az;

        // use high proportional gain to quickly converge
        q0_last = q0;
        q1_last = q1;
        q2_last = q2;
        q3_last = q3;
        
        MahonyAHRSupdateIMU(0, 0, 0, axf, ayf, azf);

        // wait for accel data to refresh
        delay(10);
    }

    // map earth frame g to sensor frame
    Quaternion g_e = Quaternion(0.0, 0, 0, 1.0);
    Quaternion q_s_to_e = Quaternion(q0, q1, q2, q3);
    Quaternion q_e_to_s = Quaternion::conjugate(q_s_to_e);
    Quaternion g_s = Quaternion::rotate(g_e, q_e_to_s);
    gs_x = g_s.getQ1();
    gs_y = g_s.getQ2();
    gs_z = g_s.getQ3();

    Serial.print("sensor grav x y z = ");
    Serial.print(gs_x, 4); Serial.print("  ");
    Serial.print(gs_y, 4); Serial.print("  ");
    Serial.print(gs_z, 4); Serial.print("  ");
    Serial.println();

    // commence accumulation: 200 samples
    Y_x_accumulated = 0;
    Y_y_accumulated = 0;
    Y_z_accumulated = 0;
    W_x_accumulated = 0;
    W_y_accumulated = 0;
    W_z_accumulated = 0;
    W_o_accumulated = 0;

    for (int i = 0; i < 200; i++){

        Serial.print("interation: "); Serial.print(i);
        Serial.println();
        accelMag.getAcceleration(&ax, &ay, &az);
        Serial.println("Accerlarations Acquired");
        axf = -0.000061035F*ax;
        ayf = -0.000061035F*ay;
        azf = -0.000061035F*az;

        Serial.println("Scaling Completed");

        Y_x_accumulated += gs_x;
        Y_y_accumulated += gs_y;
        Y_z_accumulated += gs_z;
        W_x_accumulated += axf;
        W_y_accumulated += ayf;
        W_z_accumulated += azf;
        W_o_accumulated += 1;

        Serial.println("Accumulation Completed - about to delay");

        // wait for accel data to refresh
        delay(10);
    }

    Y_matrix[position][0] = Y_x_accumulated;
    Y_matrix[position][1] = Y_y_accumulated;
    Y_matrix[position][2] = Y_z_accumulated;
    W_matrix[position][0] = W_x_accumulated;
    W_matrix[position][1] = W_y_accumulated;
    W_matrix[position][2] = W_z_accumulated;
    W_matrix[position][3] = W_o_accumulated;
    
}

void computeCalibrationConstants(){
    MatrixInst.Print((float*)W_matrix, NUM_POSITIONS, 4, "W");
    MatrixInst.Print((float*)Y_matrix, NUM_POSITIONS, 3, "Y");


    MatrixInst.Transpose((float*)W_matrix, NUM_POSITIONS, 4, (float*)Wt_matrix);
    MatrixInst.Multiply((float*)Wt_matrix, (float*)W_matrix, 4, NUM_POSITIONS, 4, (float*)WtW_matrix);
    // WtW_inv_matrix = WtW_matrix;
    // MatrixInst.Invert((float*)WtW_inv_matrix, 4);
    // MatrixInst.Multiply((float*)WtW_inv_matrix, (float*)Wt_matrix, 4, 4, NUM_POSITIONS, (float*)WtW_inv_Wt_matrix);
    MatrixInst.Invert((float*)WtW_matrix, 4);
    MatrixInst.Multiply((float*)WtW_matrix, (float*)Wt_matrix, 4, 4, NUM_POSITIONS, (float*)WtW_inv_Wt_matrix);
    MatrixInst.Multiply((float*)WtW_inv_Wt_matrix, (float*)Y_matrix, 4, NUM_POSITIONS, 3, (float*)X_matrix);
    
}


void loop(){

    for (int j = 0; j < NUM_POSITIONS; j++){
        getPositionData(j);

        //blink 10 times LED to notify user to flip
        for (int k = 0; k < 10; k++){
            digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
            delay(10);              // wait for a second
            digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
            delay(10);              // wait for a second
        }

        //wait for key press by user to continue
        Serial.println("Punch a key to continue");
        while (not Serial.available()){
            delay(10);
        }
        while (Serial.available()){
            Serial.read();
        }
    }

    computeCalibrationConstants();

    MatrixInst.Print((float*)X_matrix, 4, 3, "X");

    while (true){
        delay(1000);
    }
}