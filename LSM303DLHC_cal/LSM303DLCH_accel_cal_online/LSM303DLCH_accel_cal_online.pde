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
        // Serial.print("q0e ");  Serial.print(abs(q0_last - q0),14);
        // Serial.print(" q1e "); Serial.print(abs(q1_last - q1),14);
        // Serial.print(" q2e "); Serial.print(abs(q2_last - q2),14);
        // Serial.print(" q3e "); Serial.print(abs(q3_last - q3),14);
        // Serial.println();
        return false;
    }
}

void getPositionData(int position){
    Serial.print("Position number= ");
    Serial.println(position);
    q0_last = 1;
    q1_last = 1;
    q2_last = 1;
    q3_last = 1;

    Serial.println("Inside getPositionData");
        // wait for stable quaternion
    int l = 0;
    while (not stableQuaternion()){
        accelMag.getAcceleration(&ax, &ay, &az);
        float x_f = 0.000061035F*ax;
        float y_f = 0.000061035F*ay;
        float z_f = 0.000061035F*az;

        float xcal = x_f * acc11 + y_f * acc12 + z_f * acc13 + acc10;
        float ycal = x_f * acc21 + y_f * acc22 + z_f * acc23 + acc20;
        float zcal = x_f * acc31 + y_f * acc32 + z_f * acc33 + acc30;

        // use high proportional gain to quickly converge
        q0_last = q0;
        q1_last = q1;
        q2_last = q2;
        q3_last = q3;
        
        MahonyAHRSupdateIMU(0, 0, 0, -1.0*xcal, -1.0*ycal, -1.0*zcal);

        // wait for accel data to refresh
        delay(10);
        //Serial.println(l);
        l++;
    }
    Serial.print("Iterations until Stable:");
    Serial.println(l);

    // map earth frame g to sensor frame
    Quaternion g_e = Quaternion(0.0, 0, 0, 1.0);
    Quaternion q_s_to_e = Quaternion(q0, q1, q2, q3);
    Quaternion q_e_to_s = Quaternion::conjugate(q_s_to_e);
    Quaternion g_s = Quaternion::rotate(g_e, q_e_to_s);
    gs_x = g_s.getQ1();
    gs_y = g_s.getQ2();
    gs_z = g_s.getQ3();

    Serial.print("Y[xyz] = [");
    Serial.print(gs_x, 12); Serial.print("\t");
    Serial.print(gs_y, 12); Serial.print("\t");
    Serial.print(gs_z, 12); Serial.print("\t");
    Serial.print("]\t");

    // commence accumulation: 200 samples
    Y_x_accumulated = 0;
    Y_y_accumulated = 0;
    Y_z_accumulated = 0;
    W_x_accumulated = 0;
    W_y_accumulated = 0;
    W_z_accumulated = 0;
    W_o_accumulated = 0;

    for (int i = 0; i < 200; i++){

        // Serial.print("interation: "); Serial.print(i);
        // Serial.println();
        accelMag.getAcceleration(&ax, &ay, &az);
        // Serial.println("Accerlarations Acquired");
        // TODO: I think these should be positive to match the axes of gs_xyz
        float axf = -0.000061035F*ax;
        float ayf = -0.000061035F*ay;
        float azf = -0.000061035F*az;

        // Serial.println("Scaling Completed");

        Y_x_accumulated += gs_x;
        Y_y_accumulated += gs_y;
        Y_z_accumulated += gs_z;
        W_x_accumulated += axf;
        W_y_accumulated += ayf;
        W_z_accumulated += azf;
        W_o_accumulated += 1;

        // Serial.println("Accumulation Completed - about to delay");

        // wait for accel data to refresh
        delay(10);
    }
    Serial.print("w[xyz1] = [");
    Serial.print(W_x_accumulated/200.0, 12); Serial.print("\t");
    Serial.print(W_y_accumulated/200.0, 12); Serial.print("\t");
    Serial.print(W_z_accumulated/200.0, 12); Serial.print("\t");
    Serial.print(W_o_accumulated/200.0, 12); Serial.print("\t");
    Serial.println("]");



    Y_matrix[position][0] = Y_x_accumulated;
    Y_matrix[position][1] = Y_y_accumulated;
    Y_matrix[position][2] = Y_z_accumulated;
    W_matrix[position][0] = W_x_accumulated;
    W_matrix[position][1] = W_y_accumulated;
    W_matrix[position][2] = W_z_accumulated;
    W_matrix[position][3] = W_o_accumulated;
    
}

void computeCalibrationConstants(){
    // MatrixInst.Print((float*)W_matrix, NUM_POSITIONS, 4, "W");


    // MatrixInst.Print((float*)Y_matrix, NUM_POSITIONS, 3, "Y");

    //DEBUG
    Serial.println("Y=");
    MatrixInst.Print((float*)Y_matrix, NUM_POSITIONS, 3);
    //\DEBUG
    
    
    MatrixInst.Transpose((float*)W_matrix, NUM_POSITIONS, 4, (float*)Wt_matrix);
    //DEBUG
    Serial.println("W'=");
    MatrixInst.Print((float*)WtW_matrix, 4, NUM_POSITIONS);
    //\DEBUG
    MatrixInst.Multiply((float*)Wt_matrix, (float*)W_matrix, 4, NUM_POSITIONS, 4, (float*)WtW_matrix);
    //DEBUG
    Serial.println("W'*W=");
    MatrixInst.Print((float*)WtW_matrix, 4, 4);
    //\DEBUG
    

    MatrixInst.Invert((float*)WtW_matrix, 4);
    //DEBUG
    Serial.println("(W'*W)^(-1)=");
    MatrixInst.Print((float*)WtW_matrix, 4, 4);
    //\DEBUG
    
    MatrixInst.Multiply((float*)WtW_matrix, (float*)Wt_matrix, 4, 4, NUM_POSITIONS, (float*)WtW_inv_Wt_matrix);
    //DEBUG
    Serial.println("(W'*W)^(-1) * W'=");
    MatrixInst.Print((float*)WtW_inv_Wt_matrix, 4, NUM_POSITIONS);
    //\DEBUG
    
    MatrixInst.Multiply((float*)WtW_inv_Wt_matrix, (float*)Y_matrix, 4, NUM_POSITIONS, 3, (float*)X_matrix);
    //DEBUG
    Serial.println("(W'*W)^(-1) * W' * Y=X= ");
    MatrixInst.Print((float*)X_matrix, 4, 3);
    //\DEBUG
    
    
    // I think this is wrong:
    // acc11 = X_matrix[1][1];
    // acc12 = X_matrix[1][2];
    // acc13 = X_matrix[1][3];
    // acc10 = X_matrix[1][4];
    // acc21 = X_matrix[2][1];
    // acc22 = X_matrix[2][2];
    // acc23 = X_matrix[2][3];
    // acc20 = X_matrix[2][4];
    // acc31 = X_matrix[3][1];
    // acc32 = X_matrix[3][2];
    // acc33 = X_matrix[3][3];
    // acc30 = X_matrix[3][4];

    // I think it should be:
    // acc11 = X_matrix[1][1];
    // acc12 = X_matrix[2][1];
    // acc13 = X_matrix[3][1];
    // acc10 = X_matrix[4][1];
    // acc21 = X_matrix[1][2];
    // acc22 = X_matrix[2][2];
    // acc23 = X_matrix[3][2];
    // acc20 = X_matrix[4][2];
    // acc31 = X_matrix[1][3];
    // acc32 = X_matrix[2][3];
    // acc33 = X_matrix[3][3];
    // acc30 = X_matrix[4][3];

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

    // experiment forcing orthonality of sensor:
    acc12 = 0;
    acc13 = 0;
    acc21 = 0;
    acc23 = 0;
    acc31 = 0;
    acc32 = 0;
    



    Serial.print("acc11= "); Serial.print(acc11,6); Serial.print("\t");
    Serial.print("acc21= "); Serial.print(acc21,6); Serial.print("\t");
    Serial.print("acc31= "); Serial.print(acc31,6); Serial.print("\n");
    
    Serial.print("acc12= "); Serial.print(acc12,6); Serial.print("\t");
    Serial.print("acc22= "); Serial.print(acc22,6); Serial.print("\t");
    Serial.print("acc32= "); Serial.print(acc32,6); Serial.print("\n");
    
    Serial.print("acc13= "); Serial.print(acc13,6); Serial.print("\t");
    Serial.print("acc23= "); Serial.print(acc23,6); Serial.print("\t");
    Serial.print("acc33= "); Serial.print(acc33,6); Serial.print("\n");
    
    Serial.print("acc10= "); Serial.print(acc10,6); Serial.print("\t");
    Serial.print("acc20= "); Serial.print(acc20,6); Serial.print("\t");
    Serial.print("acc30= "); Serial.print(acc30,6); Serial.print("\n");
    
}


void loop(){

    for (int k = 0; k < 2; k++){
        // see if the quaternion converges more quickly and or increase the epsilon value
        // for the second iteration.  See how tightly you can calibrate.
        // may want to change ganes for slower but tighter convergence.
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
    }

    //MatrixInst.Print((float*)X_matrix, 4, 3, "X");

    while (true){
        delay(1000);
        accelMag.getAcceleration(&ax, &ay, &az);
        float x_f = 0.000061035F*ax;
        float y_f = 0.000061035F*ay;
        float z_f = 0.000061035F*az;

        float xcal = x_f * acc11 + y_f * acc12 + z_f * acc13 + acc10;
        float ycal = x_f * acc21 + y_f * acc22 + z_f * acc23 + acc20;
        float zcal = x_f * acc31 + y_f * acc32 + z_f * acc33 + acc30;

        Serial.print("xyz = ");
        Serial.print(xcal, 8); Serial.print("/t");
        Serial.print(ycal, 8); Serial.print("/t");
        Serial.print(zcal, 8); Serial.print("/t");
        Serial.println();
    }
}