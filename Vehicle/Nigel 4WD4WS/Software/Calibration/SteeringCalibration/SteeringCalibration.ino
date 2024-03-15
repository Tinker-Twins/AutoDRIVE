#include <Servo.h>
int servoPin = 7;
Servo steeringServo;
int zeroSteerAngle = 97; // Calibrated zero-steer angle
int steeringAngle = zeroSteerAngle; // Initialize steering angle to zero-steer angle

void setup()
{
  Serial.begin(115200); // Initialize the serial port
  Serial.setTimeout(1000000); // Set 1000 seconds timeout for serial communication
  steeringServo.write(zeroSteerAngle); // Initialize servo on power-up to zero-steer angle
  steeringServo.attach(servoPin); // Initialize the steering actuator
}

void loop()
{
  // Do only if serial communication is established
  if (Serial.available() > 0)
  {
    steeringAngle = Serial.parseInt(); // Read incoming steering angle (do not vary consecutive values by more than 60 deg)
    Serial.println(steeringAngle); // Print the steering angle on serial monitor
    steeringServo.write(steeringAngle); // Command the servo to reach the position
    delay(15); // Wait for servo to reach the commanded position
  }
}
