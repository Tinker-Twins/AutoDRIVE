#include <Servo.h>
int servoPin = 9;
Servo steeringServo;
int zeroSteerAngle = 97; // Calibrated zero-steer angle

void setup()
{
  steeringServo.write(zeroSteerAngle); // Initialize servo on power-up to zero-steer angle
  steeringServo.attach(servoPin); // Initialize the steering actuator
}

void loop()
{
  steeringServo.write(zeroSteerAngle);
  delay(1000);
  steeringServo.write(zeroSteerAngle-30);
  delay(1000);
  steeringServo.write(zeroSteerAngle);
  delay(1000);
  steeringServo.write(zeroSteerAngle+30);
  delay(1000);
}
