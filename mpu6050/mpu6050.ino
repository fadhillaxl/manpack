#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <math.h>

Adafruit_MPU6050 mpu;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }
}

void loop() {
  sensors_event_t accelEvent, gyroEvent;

  mpu.getEvent(&accelEvent, &gyroEvent);

  // Convert raw gyro values to degrees per second
  float gx_dps = gyroEvent.gyro.x / 131.0;
  float gy_dps = gyroEvent.gyro.y / 131.0;
  float gz_dps = gyroEvent.gyro.z / 131.0;

  // Calculate angles based on time
  static unsigned long prevTime = 0;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0; // Convert to seconds

  // Integrate angular rates to get angles
  static float angleX = 0.0;
  static float angleY = 0.0;
  static float angleZ = 0.0;

  angleX += gx_dps * deltaTime;
  angleY += gy_dps * deltaTime;
  angleZ += gz_dps * deltaTime;

  prevTime = currentTime;

  Serial.print("Gyroscope Angles (degrees): ");
  Serial.print("X = "); Serial.print(angleX);
  Serial.print(" Y = "); Serial.print(angleY);
  Serial.print(" Z = "); Serial.println(angleZ);

  delay(100); // Adjust delay as needed
}
