#include <Stepper.h>
#include <Ticker.h>
#include <PID_v1.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <Kalman.h>

void mpurun();
Stepper myStepperY(200*16, 4, 2);
Ticker timer5(mpurun, 10, 0);
// Ticker timer2(printCounter, 1000, 0, MILLIS);


double Setpoint, Input, Output;
double SetpointX, InputX, OutputX;



MPU6050 mpu(Wire);
double Kp=2, Ki=0.1, Kd=0.01;
PID myPIDY(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPIDX(&InputX, &OutputX, &SetpointX, Kp, Ki, Kd, DIRECT);

Kalman kalmanY;
Kalman kalmanZ;
float kalman_Y;
float kalman_Z;
// float kalman;

long timer = 0;

float kalman(double Y){
  // Define the input and output range for the map function
const int inputValueMin = (Y > 0) ? 0 : -9000;
const int inputValueMax = (Y > 0) ? 9000 : 0;
const int outputValueMin = (Y > 0) ? 9000 : 18000;
const int outputValueMax = (Y > 0) ? 0 : 9000;

// Map the input value to the output range
float kalman = map(Y * 100, inputValueMin, inputValueMax, outputValueMin, outputValueMax);
return kalman / 100;
// Print the result
// Serial.println(kalman / 100);

}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  myStepperY.setSpeed(60);

  kalmanY.setAngle(0); //set 0 angle kalmanY
  kalmanZ.setAngle(0); //set 0 angle kalmanX
 

  timer5.start();

  Input = kalman_Y;
  Setpoint = 82.00;
  myPIDY.SetMode(AUTOMATIC);
  
}

void menu1(int choice){
  while (Serial.available()) {
          // Wait for input
        }
  // int choice = Serial.parseInt();

      switch (choice) {
        case 1:
          Serial.println("Tentukan sudut Y : ");
          // Perform action for Option 1
          while (!Serial.available()) {
           // Wait for input
          }
          Setpoint = Serial.parseFloat();
          break;
        case 2:
          Serial.println("Tampilan sudut saat ini");
          // Perform action for Option 2
          Serial.print("sudut saat ini : ");
          Serial.println(Setpoint);
          break;
        case 3:
          Serial.println("Kp");
          while (!Serial.available()) {
           // Wait for input
          }
          // Perform action for Option 3
          Kp = Serial.parseFloat();
          
          break;
        case 4:
          Serial.println("Ki");
          while (!Serial.available()) {
           // Wait for input
          }
          // Perform action for Option 3
          Ki = Serial.parseFloat();

          break;
        case 5:
          Serial.println("Kd");
          while (!Serial.available()) {
           // Wait for input
          }
          // Perform action for Option 3
          Kd = Serial.parseFloat();

          break;
        case 6:
          Serial.println("Reset MPU");
          // Perform action for Option 3
          resetmpu();
          break;
        case 7:
          Serial.println("Exiting the menu.");
          // Perform any necessary cleanup and exit the menu
          timer5.start();

          break;
        default:
          Serial.println("Invalid choice. Please enter a valid option.");
      }

}

void resetmpu(){
      Wire.begin();
  
      byte status = mpu.begin();
      Serial.print(F("MPU6050 status: "));
      Serial.println(status);
      while(status!=0){ } // stop everything if could not connect to MPU6050
      
      Serial.println(F("Calculating offsets, do not move MPU6050"));
      delay(1000);
      mpu.calcOffsets(true,true); // gyro and accelero
      Serial.println("Done!\n");
      kalmanY.setAngle(0); //set 0 angle kalmanY
      kalmanZ.setAngle(0); //set 0 angle kalmanX
}

void mpurun(){
    mpu.update();
    double Y = kalmanY.getAngle(mpu.getAngleY(), mpu.getAccAngleY() / 131.0, 0.01);

    // kalman_Y = map(kalman_Y * 100, 0, 9000, 9000, 0);
    kalman_Y = kalman(Y);
    Input = kalman_Y;
    myPIDY.Compute();
    Serial.print("kalman Y : ");
    Serial.print(kalman_Y);
    Serial.print("\tkalman Z : ");
    Serial.print(mpu.getAngleZ());
    Serial.print("\tstep PID Y : ");
    const int inputstepper = (kalman_Y > Setpoint) ? Output : -Output;
    // const int inputstepper = (kalman_Y > 0) ? -inputstepper : inputstepper;

    if(kalman_Y < 0){
      const int stepper = inputstepper * -1;
      Serial.println(stepper);
      myStepperY.step(stepper);
    }else{
      const int stepper = inputstepper;
      Serial.println(stepper);
      myStepperY.step(stepper);
    }


    



}

void loop() {

    timer5.update();
    if (Serial.available()) {
    timer5.stop();
    Serial.println("Menu:");
    Serial.println("1. set Seetpoint");
    Serial.println("2. see setpoint");
    Serial.println("3. reset MPU");
    Serial.println("4. KP");
    Serial.println("5. KI");
    Serial.println("6. KD");
    Serial.println("7. Exit");
    Serial.print("Enter the number of your choice: ");

    while (!Serial.available()) {
        // Wait for input
    }

    // Read the user's choice as a string
    String input = Serial.readStringUntil('\n');
    
    // Convert the string to an integer
    int choice = input.toInt();

    // Check if the input is a valid integer
    if (choice >= 1 && choice <= 7) {
        // Call the appropriate function based on the choice
        menu1(choice);
    } else {
        Serial.println("Invalid choice. Please enter a number between 1 and 4.");
        // timer5.start();

    }
}

    
    
    
    // kalman_Z = kalmanZ.getAngle(mpu.getAngleZ(), mpu.getAccZ() / 131.0, 0.01);
    // Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
    // Serial.print("\tY: ");Serial.print(mpu.getAngleY());
    // Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());

    // Serial.print(F("ANGLE Kalman    X: "));Serial.print(Y);
    // Serial.print("\tY: ");Serial.print(kalman_Z);
    // Serial.print("\tZ: ");Serial.println(kalman_Z);
    // Serial.println(F("=====================================================\n"));
    // delay(100);

}
