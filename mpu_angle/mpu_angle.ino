#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_MPU6050 mpu;

double elevationAngle = 0.0;
double polarization = 0.0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }
  // mpu.setAccelerometerRange(MPU6050_RANGE_2_G); // Set accelerometer range
  // calibrateSensor();
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

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'r') {
      delay(5000);
      resetAngles();
      Serial.println("Angles reset to zero.");
      Serial.print("Elevation Angle: ");
      Serial.print(elevationAngle, 2);
      Serial.print(" degrees | ");

      Serial.print("Polarization: ");
      Serial.print(polarization, 2);
      Serial.println(" degrees");
      delay(5000);

    }
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  double roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / M_PI;
  double pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / M_PI;

  elevationAngle = pitch;
  polarization = 90.0 - roll;

  Serial.print("Elevation Angle: ");
  Serial.print(elevationAngle, 2);
  Serial.print(" degrees | ");

  Serial.print("Polarization: ");
  Serial.print(polarization, 2);
  Serial.println(" degrees");

  delay(500);
}

void resetAngles() {
  elevationAngle = 0.0;
  polarization = 0.0;
}

void calibrateMPU6050() {
  Serial.println("Calibrating MPU6050...");

  // Collect and average readings for offset calculation
  const int numReadings = 1000;
  for (int i = 0; i < numReadings; i++) {
 

    accel_offsets[0] += a.acceleration.x;
    accel_offsets[1] += a.acceleration.y;
    accel_offsets[2] += a.acceleration.z;
    
    gyro_offsets[0] += gyro_event.gyro.x;
    gyro_offsets[1] += gyro_event.gyro.y;
    gyro_offsets[2] += gyro_event.gyro.z;

    delay(10);
  }

  // Calculate the averages
  for (int i = 0; i < 3; i++) {
    accel_offsets[i] /= numReadings;
    gyro_offsets[i] /= numReadings;
  }

  Serial.println("Calibration complete.");
}
