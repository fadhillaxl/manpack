/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees
 * 
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>
#include <AccelStepper.h> 
#define dirPin 2 // pin yang terhubung ke DIR+ motor driver
#define ena 15 // pin yang terhubung ke DIR+ motor driver
#define stepPin 4 // pin yang terhubung ke PUL+ motor driver
AccelStepper stepper = AccelStepper(1, stepPin, dirPin);

int pinpul = 4;
int pindir = 2;
int enable = 15;
int pointDegree = 70;

MPU6050 mpu(Wire);

unsigned long interval=10; // the time we need to wait
unsigned long previousMillis=0; // millis() returns an unsigned long.
unsigned long interval2=10; // the time we need to wait
unsigned long previousMillis2=0; // millis() returns an unsigned long.
double x,y,z;
float yy;

void resetAngles(){
  x=0.00;
  y=0.00;
  z=0.00;
}

void setup() {
  Serial.begin(9600);


  Wire.begin();
  pinMode(pindir,OUTPUT);
  pinMode(pinpul,OUTPUT);
  pinMode(enable,OUTPUT);
  digitalWrite(enable, LOW);
  digitalWrite(pindir,HIGH);
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  stepper.setMaxSpeed(2000); // atur kecepatan, dapat docoba dirubah untuk latihan
  stepper.setAcceleration(500); // nilai akselerasi / percepatan, dapat docoba dirubah untuk latihan

  stepper.moveTo(1000); // beri target 360*
  stepper.runToPosition(); //jalankan 
  stepper.moveTo(-1000); // beri target 360*
  stepper.runToPosition(); //jalankan 



  Serial.println("Done!\n");
 
}

void loop() {
  mpu6050();
  	  if(yy < pointDegree){
        for(int i = 0; yy == pointDegree;i++){
          stepper.moveTo(i); // beri target 360*
          stepper.runToPosition(); //jalankan
        }
         
    } else if (yy > pointDegree) {
      for(int i = 0; yy == pointDegree;i++){
          stepper.moveTo(-i); // beri target 360*
          stepper.runToPosition(); //jalankan
        }
    }
  // stepper();
}

void mpu6050() {
  mpu.update();
   unsigned long currentMillis = millis(); // grab current time
  
 // check if "interval" time has passed (1000 milliseconds)
 if ((unsigned long)(currentMillis - previousMillis) >= interval) {
   
   	Serial.print("X : ");
  x = mpu.getAngleX();
	Serial.print(x);
	Serial.print("\tY : ");
  y = mpu.getAngleY();
  yy = map(y,-90.00,90.00,180.00,0.00);

	Serial.print(yy);
	Serial.print(y);
	Serial.print("\tZ : ");
  z = mpu.getAngleZ();
	Serial.println(z);

    if (Serial.available() > 0) {
    int command = Serial.read();
      pointDegree = command;

   
  }

  // Serial.print("\tY : ");
	// Serial.println(mpu.getAngleY());
   // save the "current" time
   previousMillis = millis();
 }


}
