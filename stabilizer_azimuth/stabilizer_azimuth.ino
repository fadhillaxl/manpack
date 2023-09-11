#include <Stepper.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <Kalman.h>


Kalman kalmanX;
Kalman kalmanY;
const int stepsPerRevolution = 200*4;  // change this to fit the number of steps per revolution
int dir = 13;

MPU6050 mpu(Wire);

Stepper myStepper(stepsPerRevolution, 14, 12);
const int delayMS = 0;


void setup() {
  // set the speed at 60 rpm:
  pinMode(dir,OUTPUT);
  digitalWrite(dir,LOW);
  Serial.begin(9600);
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


}

void loop() {
  // step one revolution in one direction:
  mpu.update();
  double kalman = kalmanY.getAngle(mpu.getAngleY(),mpu.getAccAngleY()/131.0,0.01);
  double Y = map(mpu.getAngleY()*100,0,9000,9000,0);
  // Serial.print("angle : ");
  Serial.println(kalman);
  // Serial.println(mpu.getAngleY());
  if(int(mpu.getAngleY()) > -20){
    myStepper.step(17);
  }else if(int(mpu.getAngleY()) < -20){
    myStepper.step(-17);
  }
  delay(10);

}