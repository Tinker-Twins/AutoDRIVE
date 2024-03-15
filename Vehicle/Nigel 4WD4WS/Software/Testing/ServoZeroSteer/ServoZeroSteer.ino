#include <Servo.h>

int servoFLPin = 7;
int servoFRPin = 6;
int servoRLPin = 4;
int servoRRPin = 5;

Servo servoFL;
Servo servoFR;
Servo servoRL;
Servo servoRR;

int zeroSteerFL = 90; // Calibrated zero-steer angle
int zeroSteerFR = 90; // Calibrated zero-steer angle
int zeroSteerRL = 90; // Calibrated zero-steer angle
int zeroSteerRR = 90; // Calibrated zero-steer angle

void setup()
{
  servoFL.write(zeroSteerFL); // Initialize servo on power-up to zero-steer angle
  servoFR.write(zeroSteerFR); // Initialize servo on power-up to zero-steer angle
  servoRL.write(zeroSteerRL); // Initialize servo on power-up to zero-steer angle
  servoRR.write(zeroSteerRR); // Initialize servo on power-up to zero-steer angle
  servoFL.attach(servoFLPin); // Initialize the steering actuator
  servoFR.attach(servoFRPin); // Initialize the steering actuator
  servoRL.attach(servoRLPin); // Initialize the steering actuator
  servoRR.attach(servoRRPin); // Initialize the steering actuator
}

void loop()
{
  // Zero Steering
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
}
