#include <Stepper.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <Kalman.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

SoftwareSerial serial_gps(17, 16);
TinyGPSPlus gps;
unsigned long GPSStartMillis;
unsigned long currentMillis;
const unsigned long GPSPeriod = 3000;   //GPS period

float Longitude = 0;
float Latitude = 0;
float Speed = 0;
float Altitude = 0;


Kalman kalmanX;
Kalman kalmanY;
const int stepsPerRevolution = 200*4;  // change this to fit the number of steps per revolution
int dir = 13;
const int puly = 4;
const int diry = 2;
const int pulx = 19;
const int dirx = 18;
const int limEU = 13;
const int limED = 12;
const int limAZ = 14;
MPU6050 mpu(Wire);

Stepper myStepper(stepsPerRevolution, puly, diry);
Stepper myStepperX(stepsPerRevolution, pulx, dirx);
const int delayMS = 0;


void kalibrasi(){
  while (limEU != HIGH)
  {
    if(limED != HIGH){
      myStepper.step(1);

    }else{
      myStepper.step(-1);

    }
  }

  Serial.println("Kalibrasi Done!\n");
  
}

void CheckGPS() {
  while (serial_gps.available()) {
    gps.encode(serial_gps.read());
  }
}

void FunctionGPS() {
  if (currentMillis - GPSStartMillis >= GPSPeriod)  //test whether the period has elapsed
  {
    if (gps.location.isUpdated()) {
      Latitude = gps.location.lat();
      Longitude = gps.location.lng();
      Speed = gps.speed.kmph();
      Altitude = gps.altitude.meters();
      Serial.println("");
      Serial.println("");

      Serial.println("Re-Checking GPS Signal");
      Serial.print("Latitude= ");
      Serial.println(Latitude, 6);

      Serial.print("Longitude= ");
      Serial.println(Longitude, 6);

      Serial.print("Altitude(meters)= ");
      Serial.println(Altitude, 2);

      Serial.print("SPEED(KMPH)= ");
      Serial.println(Speed, 2);
    }
    GPSStartMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
  }
}

void setgps(){
  serial_gps.begin(9600);
  Serial.println("GPS Mulai");
  GPSStartMillis = millis();    //initial start time

}

void setup() {
  // set the speed at 60 rpm:
  pinMode(limEU,OUTPUT);
  pinMode(limED,OUTPUT);
  pinMode(limAZ,OUTPUT);
  Serial.begin(9600);
  kalibrasi();
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(10000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  myStepper.setSpeed(180);
  kalmanX.setAngle(0);
  kalmanY.setAngle(0);

  setgps();


}

void loop() {
  // step one revolution in one direction:
  mpu.update();
  double kalman_Y = kalmanY.getAngle(mpu.getAngleY(),mpu.getAccAngleY()/131.0,0.01);
  double Y = map(kalman_Y*100,0,9000,9000,0);
  // Serial.print("angle : ");
  Serial.println(Y);
  // Serial.println(mpu.getAngleY());
  if(int(Y) > 82){
    myStepper.step(17);
  }else if(int(Y) < 82){
    myStepper.step(-17);
  }
  delay(10);

}