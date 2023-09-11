#include <Stepper.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <Kalman.h>
#include <PID_v1.h>

Kalman kalmanX;
Kalman kalmanY;
const int stepsPerRevolution = 200 * 4;  // Change this to fit the number of steps per revolution
int dir = 13;

MPU6050 mpu(Wire);

Stepper myStepper(stepsPerRevolution, 14, 12);
const int delayMS = 0;

// Define PID constants
double Kp = 1.0;  // Proportional constant
double Ki = 0.0;  // Integral constant
double Kd = 0.0;  // Derivative constant

double setpoint = 0.0;  // Desired angle
double stepy;
double kalman_y;

// Create a PID object
PID myPID(&kalman_y, &stepy, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // Set the speed at 60 RPM:
  pinMode(dir, OUTPUT);
  digitalWrite(dir, LOW);
  Serial.begin(9600);
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}  // Stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(10000);
  mpu.calcOffsets(true, true);  // Gyro and accelerometer calibration
  Serial.println("Done!\n");
  myStepper.setSpeed(180);
  kalmanX.setAngle(0);
  kalmanY.setAngle(0);

  // Initialize the PID controller
  myPID.SetMode(AUTOMATIC);
  // myPID.SetOutputLimits(-200, 200);  // Adjust these limits as needed
}

void loop() {
  // Step one revolution in one direction:
  mpu.update();
  double kalman = kalmanY.getAngle(mpu.getAngleY(), mpu.getAccAngleY() / 131.0, 0.01);
  double Y = map(mpu.getAngleY() * 100, 0, 9000, 9000, 0);

  kalman_y = kalman;

  // Set the setpoint to the desired angle (adjust as needed)
  setpoint = 0.0;

  // Compute the PID control output
  myPID.Compute();

  // Print the filtered angle and PID output
  Serial.print("Angle: ");
  Serial.print(kalman);
  Serial.print("  PID Output: ");
  Serial.println(stepy);

  // Add a condition for stopping the motor (e.g., if you reach a desired angle)

  delay(10);
}

