#include <Stepper.h>
#include <Ticker.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Kalman.h>
#include <PID_v1.h>

class MyMPU6050 {
public:
  MyMPU6050() : mpu(Wire) {}
  
  void initialize() {
    Wire.begin();
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0) { } // stop everything if could not connect to MPU6050

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true, true); // gyro and accelero
    Serial.println("Done!\n");
  }

  float getYAngle() {
    mpu.update();
    return kalmanY.getAngle(mpu.getAngleY(), mpu.getAccAngleY() / 131.0, 0.01);
  }
  
  void reset() {
    Wire.begin();
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0) { } // stop everything if could not connect to MPU6050

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true, true); // gyro and accelero
    Serial.println("Done!\n");
    kalmanY.setAngle(0); // set 0 angle kalmanY
    kalmanZ.setAngle(0); // set 0 angle kalmanX
  }

  float kalman(double Y) {
    // Define the input and output range for the map function
    const int inputValueMin = (Y > 0) ? 0 : -9000;
    const int inputValueMax = (Y > 0) ? 9000 : 0;
    const int outputValueMin = (Y > 0) ? 9000 : 18000;
    const int outputValueMax = (Y > 0) ? 0 : 9000;

    // Map the input value to the output range
    float kalmanValue = map(Y * 100, inputValueMin, inputValueMax, outputValueMin, outputValueMax);
    return kalmanValue / 100;
  }

private:
  MPU6050 mpu;
  Kalman kalmanY;
  Kalman kalmanZ;
};

class MyPIDController {
public:
  MyPIDController(double setpoint, double kp, double ki, double kd)
    : setpoint(setpoint), kp(kp), ki(ki), kd(kd), input(0), output(0), myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT) {}

  void initialize() {
    myPID.SetMode(AUTOMATIC);
  }

  void setInput(double newInput) {
    input = newInput;
  }

  void setSetpoint(double newSetpoint) {
    setpoint = newSetpoint;
  }

  double compute() {
    myPID.Compute();
    return output;
  }

private:
  double setpoint;
  double kp, ki, kd;
  double input;
  double output;
  PID myPID;
};

class StepperMotor {
public:
  StepperMotor(int stepsPerRevolution, int pin1, int pin2)
    : stepper(stepsPerRevolution, pin1, pin2) {}

  void setSpeed(int speed) {
    stepper.setSpeed(speed);
  }

  void step(int steps) {
    stepper.step(steps);
  }

private:
  Stepper stepper;
};

MyMPU6050 mpuSensor;
MyPIDController pidControllerY(82.0, 2.0, 0.1, 0.01);
StepperMotor myStepperY(200 * 16, 4, 2);

void setup() {
  Serial.begin(9600);

  mpuSensor.initialize();
  pidControllerY.initialize();
  myStepperY.setSpeed(60);
}

void loop() {
  double Y = mpuSensor.getYAngle();
  double kalman_Y = kalman(Y);

  pidControllerY.setInput(kalman_Y);
  int outputY = pidControllerY.compute();

  Serial.print("kalman Y : ");
  Serial.print(kalman_Y);
  Serial.print("\tstep PID Y : ");
  Serial.println(outputY);

  if (kalman_Y < 0) {
    myStepperY.step(-outputY);
  } else {
    myStepperY.step(outputY);
  }
}
