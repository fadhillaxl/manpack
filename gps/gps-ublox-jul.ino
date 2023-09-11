#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
SoftwareSerial serial_gps(17, 16);
TinyGPSPlus gps;

unsigned long GPSStartMillis;
unsigned long currentMillis;
const unsigned long GPSPeriod = 3000;   //GPS period

float Longitude = 0;
float Latitude = 0;
float Speed = 0;
float Altitude = 0;

void mpu_setup(){
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void mpu_loop(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
}


void setup() {
  Serial.begin(9600);
  serial_gps.begin(9600);
  Serial.println("GPS Mulai");
  GPSStartMillis = millis();    //initial start time
  mpu_setup();

}

void loop() {
  CheckGPS();
  currentMillis = millis();
  FunctionGPS();
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

      mpu_loop();

    }
    GPSStartMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
  }
}

void CheckGPS() {
  while (serial_gps.available()) {
    gps.encode(serial_gps.read());
  }
}