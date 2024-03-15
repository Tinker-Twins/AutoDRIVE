#include <Servo.h>

int speedPin = 6;
int dirxnPin = 7;
int brakePin = 8;

boolean dirxnValue = 0;
float speedValue = 0;
boolean brakeValue = 0;

void setup()
{
  Serial.begin(115200); // Initialize the serial port
  driveInit(); // Initialize the motor driver
}

void loop()
{
  delay(10000); // Initialization delay

  // Drive forward
  dirxnValue = 0;
  brakeValue = 0;
  for(speedValue=0; speedValue<=256; speedValue++){
    digitalWrite(dirxnPin, dirxnValue);
    analogWrite(speedPin, speedValue);
    digitalWrite(brakePin, brakeValue);
    Serial.print("Throttle:"); Serial.println(speedValue/255);
    delay(25);
  }

  // Brake
  speedValue = 0;
  brakeValue = 1;
  analogWrite(speedPin, speedValue);
  digitalWrite(brakePin, brakeValue);
  Serial.print("Throttle:"); Serial.println(speedValue/255);

  // Drive reverse
  dirxnValue = 1;
  brakeValue = 0;
  for(speedValue=0; speedValue<=256; speedValue++){
    digitalWrite(dirxnPin, dirxnValue);
    analogWrite(speedPin, speedValue);
    digitalWrite(brakePin, brakeValue);
    Serial.print("Throttle:"); Serial.println(-speedValue/255);
    delay(25);
  }

  // Brake
  speedValue = 0;
  brakeValue = 1;
  analogWrite(speedPin, speedValue);
  digitalWrite(brakePin, brakeValue);
  Serial.print("Throttle:"); Serial.println(speedValue/255);
}

void driveInit()
{
  pinMode(speedPin, OUTPUT); // Declare speed pin
  pinMode(brakePin, OUTPUT); // Declare brake pin
  pinMode(dirxnPin, OUTPUT); // Declare direction pin
}
