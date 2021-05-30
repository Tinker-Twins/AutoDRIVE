#include <Servo.h>

int leftEncoderPinA = 2;
int leftEncoderPinB = 4;
int rightEncoderPinA = 3;
int rightEncoderPinB = 5;

int leftEncoderALast = 0;
int rightEncoderALast = 0;

int leftEncoderTicks = 0; // Number of ticks
boolean leftEncoderDirxn = true; // Rotation direction
int rightEncoderTicks = 0;// Number of ticks
boolean rightEncoderDirxn = true; // Rotation direction

int speedPin = 6;
int dirxnPin = 7;
int brakePin = 8;

boolean dirxnValue = 0;
int speedValue = 0;
boolean brakeValue = 0;

void setup()
{
  Serial.begin(115200); // Initialize the serial port
  driveInit(); // Initialize the motor driver
  encodersInit(); // Initialize the encoders
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
    Serial.print("Left-Ticks:"); Serial.print(leftEncoderTicks); Serial.print(",");Serial.print("Right-Ticks:"); Serial.println(rightEncoderTicks);
    delay(25);
  }

  // Brake
  speedValue = 0;
  brakeValue = 1;
  analogWrite(speedPin, speedValue);
  digitalWrite(brakePin, brakeValue);
  Serial.print("Left-Ticks:"); Serial.print(leftEncoderTicks); Serial.print(","); Serial.print("Right-Ticks:"); Serial.println(rightEncoderTicks);

  // Drive reverse
  dirxnValue = 1;
  brakeValue = 0;
  for(speedValue=0; speedValue<=256; speedValue++){
    digitalWrite(dirxnPin, dirxnValue);
    analogWrite(speedPin, speedValue);
    digitalWrite(brakePin, brakeValue);
    Serial.print("Left-Ticks:"); Serial.print(leftEncoderTicks); Serial.print(","); Serial.print("Right-Ticks:"); Serial.println(rightEncoderTicks);
    delay(25);
  }+

  // Brake
  speedValue = 0;
  brakeValue = 1;
  analogWrite(speedPin, speedValue);
  digitalWrite(brakePin, brakeValue);
  Serial.print("Left-Ticks:"); Serial.println(leftEncoderTicks); //Serial.print(","); Serial.print("Right-Ticks:"); Serial.println(rightEncoderTicks);
}

void driveInit()
{
  pinMode(speedPin, OUTPUT); // Declare speed pin
  pinMode(brakePin, OUTPUT); // Declare brake pin
  pinMode(dirxnPin, OUTPUT); // Declare direction pin
}

void encodersInit()
{
  leftEncoderDirxn = true; // Default rotation direction: forward
  rightEncoderDirxn = true; // Default rotation direction: forward
  pinMode(leftEncoderPinB,INPUT); // Channel B
  pinMode(rightEncoderPinB,INPUT); // Channel B
  pinMode(leftEncoderPinA, INPUT_PULLUP); // Channel A
  pinMode(rightEncoderPinA, INPUT_PULLUP); // Channel A
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), leftEncoderISR, CHANGE); // Declare interrupt
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), rightEncoderISR, CHANGE); // Declare interrupt
}

void leftEncoderISR()
{
  int leftEncoderA = digitalRead(leftEncoderPinA);
  if((leftEncoderALast == LOW) && leftEncoderA==HIGH)
  {
    int leftEncoderB = digitalRead(leftEncoderPinB);
    if(leftEncoderB == LOW && leftEncoderDirxn)
    {
      leftEncoderDirxn = false; // Reverse
    }
    else if(leftEncoderB == HIGH && !leftEncoderDirxn)
    {
      leftEncoderDirxn = true;  // Forward
    }
  }
  leftEncoderALast = leftEncoderA;

  if(leftEncoderDirxn)  leftEncoderTicks--;
  else  leftEncoderTicks++;
}

void rightEncoderISR()
{
  int rightEncoderA = digitalRead(rightEncoderPinA);
  if((rightEncoderALast == LOW) && rightEncoderA == HIGH)
  {
    int rightEncoderB = digitalRead(rightEncoderPinB);
    if(rightEncoderB == LOW && rightEncoderDirxn)
    {
      rightEncoderDirxn = false; // Reverse
    }
    else if(rightEncoderB == HIGH && !rightEncoderDirxn)
    {
      rightEncoderDirxn = true;  // Forward
    }
  }
  rightEncoderALast = rightEncoderA;

  if(rightEncoderDirxn)  rightEncoderTicks++;
  else  rightEncoderTicks--;
}
