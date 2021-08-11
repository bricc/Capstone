/************************************************************************************
 * This file is for ESP32's BNO055 Interface
 * Created by: Ricardo Bravo
 *****************************************************************************/ 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define BNO055_SAMPLERATE_DELAY_MS (10)

// CONSTANTS
static float dt;
static float q0, q1, q2, q3;
static float la_x, la_y, la_z;
static float q0dot,q1dot,q2dot,q3dot;
static float q0old = 0;float q1old = 0;float q2old = 0;float q3old = 0;
static float wx,wy,wz;
static const float pi = 3.14159;
static float wxold = 0; float wyold = 0; float wzold = 0;
static float wxnew,wynew,wznew;
static float smoothx = 0; float smoothy = 0; float smoothz = 0;
static float accx,accy,accz,accxnew,accynew,accznew;
static float accxold = 0; float accyold = 0; float acczold = 0;
static unsigned long millisOld;

// Important part of the code - 55 & 0x28 - which is the address of the I2C
Adafruit_BNO055 myIMU = Adafruit_BNO055(55,0x28);

/****************************************************************************
 * INITIALIZE BNO055 IMU 
 * 
****************************************************************************/ 
void initialize_IMU() {
  Serial.begin(115200);
  /* Initialise the sensor */
  if(!myIMU.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("No IMU detected"); 
    while(1);
  }

  // debug variables 
  millisOld=millis();
  int8_t temp=myIMU.getTemp();
  delay(1000);
  myIMU.setExtCrystalUse(true);
}


/****************************************************************************
 * READ BNO055 IMU 
 * 
****************************************************************************/

 String read_IMU() {
  uint8_t system, gyro, accel, mg = 0;
  myIMU.getCalibration(&system, &gyro, &accel, &mg);  

  /* EXTRACTING QUATERNION VALUES */ 
  imu::Quaternion quat=myIMU.getQuat();
  q0 = quat.w();
  q1 = quat.x();
  q2 = quat.y();
  q3 = quat.z();

  /* EXTRACTING LINEAR ACCELERATION FROM THE IMU */
  imu::Vector<3> linear_accel_raw = myIMU.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  la_x = linear_accel_raw.x();
  la_y = linear_accel_raw.y();
  la_z = linear_accel_raw.z();


  //Get Clock diff
  dt=(millis()-millisOld)/1000.;
  
  //get qdot
  q0dot = (q0-q0old)/dt;
  q1dot = (q1-q1old)/dt;
  q2dot = (q2-q2old)/dt;
  q3dot = (q3-q3old)/dt;

  //Find w
  wy = 2*(-q1*q0dot+q0*q1dot-q3*q2dot+q2*q3dot);
  wx = -2*(-q2*q0dot+q3*q1dot+q0*q2dot-q1*q3dot);
  wz = 2*(-q3*q0dot-q2*q1dot+q1*q2dot+q0*q3dot);
  
  //w HPF
  wxnew=.8*wxold+.2*wx;
  wynew=.8*wyold+.2*wy;
  wznew=.8*wzold+.2*wz;
  
  //Renew variables 
  millisOld=millis();
  q0old = q0;
  q1old = q1;
  q2old = q2;
  q3old = q3;
  
  //raw gyro data
//imu::Vector<3> gyr =myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  //Find Angular Acc
  accx = (wxnew-smoothx)/dt;
  accy = (wynew-smoothy)/dt;
  accz = (wznew-smoothz)/dt;
  
  //acc HPF
  accxnew=.6*accxold+.4*accx;
  accynew=.6*accyold+.4*accy;
  accznew=.6*acczold+.4*accz;
  
  wxold=wxnew;
  wyold=wynew;
  wzold=wznew;
  smoothx=wxnew;
  smoothy=wynew;
  smoothz=wznew;
  accxold=accxnew;
  accyold=accynew;
  acczold=accznew;
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
//  return String(la_x) + "," + String(la_y) + "," + String(la_z);
  return String(accxnew) + "," + String(accynew) + "," + String(accznew);
//  return (sqrt(sq(accxnew)+sq(accynew)+sq(accznew)));
}


/* DEBUG - XYZ ANGULAR ACCELERATION */ 
float return_IMUX(){
//  return accxnew >= 0? accxnew : -1;
  return abs(accxnew);
}
float return_IMUY(){
//  return accynew >= 0? accynew : -1;

return abs(accynew);
}
float return_IMUZ(){

return abs(accznew);
}

/* DEBUG - RETURN LINEAR ACCELEARATION*/

float return_LIN_IMUX(){
//  return accxnew >= 0? accxnew : -1;
  if(la_x < 0){ 
    la_x = 0;
  }
  return la_x;
}
float return_LIN_IMUY(){
//  return accynew >= 0? accynew : -1;
  if(la_y < 0){ 
    la_y = 0;
  }
return la_y;
}
float return_LIN_IMUZ(){
    if(la_z < 0){ 
    la_z = 0;
  }
  return la_z;
}

bool angularTrigger(){
  read_IMU();

   
  bool pass = false; 
  
  if(abs(accxnew) > 5 || abs(accynew) > 5 || abs(accznew) > 5){
      Serial.print(accxnew);
  Serial.print(" ");
  Serial.print(accynew);
  Serial.print(" ");
  Serial.println(accznew);
    pass = true;
  }

  return pass;
}

bool angularCalib(){
  read_IMU();

   
  bool pass = false; 
  
  if(abs(accznew) > 15){
      Serial.print(accxnew);
  Serial.print(" ");
  Serial.print(accynew);
  Serial.print(" ");
  Serial.println(accznew);
    pass = true;
  }

  return pass;
}
