#include <SparkFun_TB6612.h>
// #include <Servo.h> // This library blocks timer for D9 and D10, which is req. for PWM output

#define AIN1 A0
#define BIN1 A2
#define AIN2 13
#define BIN2 A3
#define PWMA 11
#define PWMB 10
#define STBY A1

const int offsetA = 1; // Direction offset for A channel
const int offsetB = -1; // Direction offset for B channel
Motor motorLeft = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motorRight = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
// int servoLeftPin = 5;
// int servoRightPin = 6;
// Servo servoLeft;
// Servo servoRight;
// int zeroSteerLeft = 90; // Calibrated zero-steer angle
// int zeroSteerRight = 90; // Calibrated zero-steer angle

void setup()
{
  // servoLeft.write(zeroSteerLeft); // Initialize servo on power-up to zero-steer angle
  // servoRight.write(zeroSteerRight); // Initialize servo on power-up to zero-steer angle
  // servoLeft.attach(servoLeftPin); // Initialize the steering actuator
  // servoRight.attach(servoRightPin); // Initialize the steering actuator
}


void loop()
{
  // Initialization
  brake(motorLeft, motorRight);
  delay(4000);

  // All-wheel drive
  forward(motorLeft, motorRight, 40);
  delay(7000);

  // Front-wheel drive
  forward(motorLeft, motorRight, 40);
  delay(7000);

  // Rear-wheel drive
  brake(motorLeft, motorRight);
  delay(7000);

  // Neutral-steer drive
  left(motorLeft, motorRight, 80);
  delay(7000);

  // Pivot-steer drive
  motorLeft.brake();
  motorRight.drive(40, 7000);
  
  // Torque-vector drive
  motorLeft.drive(40);
  motorRight.drive(20);
  delay(7000);
}