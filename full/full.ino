#include <PID_v1.h>
#include <Ticker.h>
#include <Stepper.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Kalman.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Sgp4.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double SetpointX, InputX, OutputX;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPIDY(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPIDX(&InputX, &OutputX, &SetpointX, Kp, Ki, Kd, DIRECT);

void FunctionGPS();
void mpu_start();
Ticker timer5(FunctionGPS, 100, 0, MILLIS);
Ticker mpu_ouput_ticker(mpu_start, 100, 0, MILLIS);

const int limEU = 13;
const int limED = 12;
const int limAZ = 14;
const unsigned long GPSPeriod = 3000;
int step_stepper = 17;

Sgp4 sat;
SoftwareSerial serial_gps(17, 16);
TinyGPSPlus gps;
unsigned long GPSStartMillis;
unsigned long currentMillis;

float Longitude = 0;
float Latitude = 0;
float Speed = 0;
float Altitude = 0;

Kalman kalmanY;
Kalman kalmanX;
double kalman_Y;
double kalman_X;
const int stepsPerRevolution = 200 * 4;
Stepper myStepper(stepsPerRevolution, 4, 2);
Stepper myStepperx(stepsPerRevolution, 18, 19);
MPU6050 mpu(Wire);

void kalibrasi() {
  while (digitalRead(limEU) != HIGH) {
    int limEDStatus = digitalRead(limED);
    myStepper.step(limEDStatus != HIGH ? 1 : -1);
  }
  Serial.println("Kalibrasi Selesai!\n");
}

void CheckGPS() {
  while (serial_gps.available()) {
    gps.encode(serial_gps.read());
  }
}

void FunctionGPS() {
  
    if (gps.location.isUpdated()) {
      Latitude = gps.location.lat();
      Longitude = gps.location.lng();
      Speed = gps.speed.kmph();
      Altitude = gps.altitude.meters();

      Serial.println("");
      Serial.println("");
      Serial.println("Memeriksa Ulang Sinyal GPS");

      Serial.print("Latitude= ");
      Serial.println(Latitude, 6);
      Serial.print("Longitude= ");
      Serial.println(Longitude, 6);
      Serial.print("Altitude (meter)= ");
      Serial.println(Altitude, 2);
      Serial.print("Kecepatan (KMPH)= ");
      Serial.println(Speed, 2);
    }
}

void setgps() {
  serial_gps.begin(9600);
  Serial.println("GPS Dimulai");
  // GPSStartMillis = millis();
  timer5.start();

}

void sgp4(){
  sat.site(Latitude, Longitude, Altitude);
  char satname[] = "ISS (ZARYA)";
  char tle_line1[] = "1 25544U 98067A   16065.25775256 -.00164574  00000-0 -25195-2 0  9990";  //Line one from the TLE data
  char tle_line2[] = "2 25544  51.6436 216.3171 0002750 185.0333 238.0864 15.54246933988812";  //Line two from the TLE data
  sat.init(satname, tle_line1, tle_line2);

}

void mpu_start(){
  Wire.begin();
  byte status = mpu.begin();

  Serial.print(F("Status MPU6050: "));
  Serial.println(status);

  while (status != 0) { }

  Serial.println(F("Menghitung Offset, jangan gerakkan MPU6050"));
  delay(10000);
  mpu.calcOffsets(true, true);
  Serial.println("Selesai!\n");
}

void output_mpu(double kalman_Y,double kalman_X){
   mpu.update();
  double Y = kalmanY.getAngle(mpu.getAngleY(), mpu.getAccAngleY() / 131.0, 0.01);
  kalman_Y = map(kalman_Y * 100, 0, 9000, 9000, 0);
  kalman_X = kalmanY.getAngle(mpu.getAngleX(), mpu.getAccAngleX() / 131.0, 0.01);
}

void PID_func(double setpoint_func,double input_func){
  Input = input_func;
  Setpoint = setpoint_func;

  //turn the PID on
  myPIDY.SetMode(AUTOMATIC);
}

void setup() {
  pinMode(limEU, OUTPUT);
  pinMode(limED, OUTPUT);
  pinMode(limAZ, OUTPUT);
  Serial.begin(9600);
  kalibrasi(); //memposisikan mpu lurus dengan titik 0 derajat elevasi
  mpu_start(); //memulai MPU6050
  mpu_ouput_ticker.start();
  myStepper.setSpeed(180); //rpm stepper
  kalmanY.setAngle(0); //set 0 angle kalmanY
  kalmanX.setAngle(0); //set 0 angle kalmanX
  setgps(); //memulai GPS
  sgp4(); //menghitung elevasi menggunakan GPS dan TLE
  // GoTo_position_TLE_Elevasi(); //memerintahkan menuju elevasi yang sudah dihitung 
  PID_func(sat.satEl,kalman_Y);
  while(sat.satEl == kalman_Y){
      Input = kalman_Y;
      myPIDY.Compute();
      myStepper.step(Output);
  }
  
  scanning_X(); //memutar AZ 4 kali
}

void scanning_Y(){

}

void scanning_X(){
  for (int x = 0; x < 5; x++) 
    {
      while (digitalRead(limAZ) == HIGH) {
          myStepper.step(17);
      }
      while (digitalRead(limAZ) != HIGH) {
          myStepper.step(-17);
      }
    }

}

void GoTo_position_TLE_Elevasi(){
  while (kalman_Y != sat.satEl) {
  if (digitalRead(limEU) != HIGH) {
    int stepDirection = (kalman_Y > sat.satEl) ? step_stepper : -step_stepper;
    myStepper.step(stepDirection);
  } else {
    if (kalman_Y < sat.satEl) {
      myStepper.step(-step_stepper);
    }
  }
}


}

void loop() {
  // mpu.update();
  // double kalman_Y = kalmanY.getAngle(mpu.getAngleY(), mpu.getAccAngleY() / 131.0, 0.01);
  // double Y = map(kalman_Y * 100, 0, 9000, 9000, 0);
  // double kalman_X = kalmanY.getAngle(mpu.getAngleX(), mpu.getAccAngleX() / 131.0, 0.01);
  // Serial.println(Y);


  




  // if (int(kalman_X) > sat.satAz) {
  //   myStepperx.step(step_stepper);
  // }
  // else if (int(kalman_X) < sat.satAz) {
  //   myStepperx.step(-step_stepper);
  // }
  delay(10);
}
