// LSM303DLCH_accel_cal_result_test.ino


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>

// I2Cdev and LSM303DLHC must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <LSM303DLHC.h>

// default address is 105
// specific I2C address may be passed here
LSM303DLHC accelMag;

int16_t ax, ay, az;
float axcal, aycal, azcal;

/* Calibration constants:
   9.8286e-01   1.0602e-02  -5.7796e-03
  -6.2650e-03   9.7135e-01  -3.0856e-03
  -5.1706e-04   2.1296e-03   9.8040e-01
   1.5158e-02   1.6440e-02  -6.7750e-02
*/

float acc11 =  0.98286000; float acc21 = 0.0106020; float acc31 = -0.0057796;
float acc12 = -0.00626500; float acc22 = 0.9713500; float acc32 = -0.0030856;
float acc13 = -0.00051706; float acc23 = 0.0021296; float acc33 =  0.9804000;
float acc10 =  0.01515800; float acc20 = 0.0164400; float acc30 = -0.0677500;



void setup() {
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

    // test scale
    Serial.print("Accel Scale: ");
    Serial.println(accelMag.getAccelFullScale());

    // test data rate
    Serial.print("Accel Output Data Rate: ");
    Serial.println(accelMag.getAccelOutputDataRate());


}

void loop() {
    // read raw angular velocity measurements from device
    accelMag.getAcceleration(&ax, &ay, &az);

    //calibrate:
    axcal = ax*acc11 + ay*acc12 + az*acc13 + acc10;
    aycal = ax*acc21 + ay*acc22 + az*acc23 + acc20;
    azcal = ax*acc31 + ay*acc32 + az*acc33 + acc30;

    Serial.print("Accelation:\t");
    Serial.print(axcal*0.0000625F,3); Serial.print("\t");
    Serial.print(aycal*0.0000625F,3); Serial.print("\t");
    Serial.print(azcal*0.0000625F,3); 
    Serial.print("\tUncal:\t");
    Serial.print(ax*0.0000625F,3); Serial.print("\t");
    Serial.print(ay*0.0000625F,3); Serial.print("\t");
    Serial.print(az*0.0000625F,3); 
    
    Serial.println();

    delay(1000);
}
