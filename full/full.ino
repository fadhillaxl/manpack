#include <Stepper.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Kalman.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Sgp4.h>

const int limEU = 13;
const int limED = 12;
const int limAZ = 14;
const unsigned long GPSPeriod = 3000;

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
  if (currentMillis - GPSStartMillis >= GPSPeriod) {
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
    GPSStartMillis = currentMillis;
  }
}

void setgps() {
  serial_gps.begin(9600);
  Serial.println("GPS Dimulai");
  GPSStartMillis = millis();
}

void sgp4(){
  sat.site(Latitude, Longitude, Altitude);
  char satname[] = "ISS (ZARYA)";
  char tle_line1[] = "1 25544U 98067A   16065.25775256 -.00164574  00000-0 -25195-2 0  9990";  //Line one from the TLE data
  char tle_line2[] = "2 25544  51.6436 216.3171 0002750 185.0333 238.0864 15.54246933988812";  //Line two from the TLE data
  sat.init(satname, tle_line1, tle_line2);

}

void setup() {
  pinMode(limEU, OUTPUT);
  pinMode(limED, OUTPUT);
  pinMode(limAZ, OUTPUT);
  Serial.begin(9600);
  kalibrasi();
  Wire.begin();
  byte status = mpu.begin();

  Serial.print(F("Status MPU6050: "));
  Serial.println(status);

  while (status != 0) { }

  Serial.println(F("Menghitung Offset, jangan gerakkan MPU6050"));
  delay(10000);
  mpu.calcOffsets(true, true);
  Serial.println("Selesai!\n");
  myStepper.setSpeed(180);
  kalmanY.setAngle(0);
  setgps();
  sgp4();

}

void loop() {
  mpu.update();
  double kalman_Y = kalmanY.getAngle(mpu.getAngleY(), mpu.getAccAngleY() / 131.0, 0.01);s
  double Y = map(kalman_Y * 100, 0, 9000, 9000, 0);
  double kalman_X = kalmanY.getAngle(mpu.getAngleX(), mpu.getAccAngleX() / 131.0, 0.01);
  Serial.println(Y);

  if (int(Y) > sat.satEl) {
    myStepper.step(17);
  }
  else if (int(Y) < sat.satEl) {
    myStepper.step(-17);
  }

  if (int(kalman_X) > sat.satAz) {
    myStepperx.step(17);
  }
  else if (int(kalman_X) < sat.satAz) {
    myStepperx.step(-17);
  }
  delay(10);
}
