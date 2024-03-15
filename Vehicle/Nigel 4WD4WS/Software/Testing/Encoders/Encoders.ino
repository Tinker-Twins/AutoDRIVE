#include <Wire.h>
#include <Servo.h>
#include <SparkFun_TB6612.h>

#define FAIN1 A10
#define FBIN1 A8
#define FAIN2 A11
#define FBIN2 A7
#define FPWMA 10
#define FPWMB 11
#define FSTBY A9

#define RAIN1 A5
#define RBIN1 A3
#define RAIN2 A6
#define RBIN2 A2
#define RPWMA 8
#define RPWMB 9
#define RSTBY A4

const int FOffsetA = -1; // Direction offset for A channel
const int FOffsetB = 1; // Direction offset for B channel
const int ROffsetA = 1; // Direction offset for A channel
const int ROffsetB = -1; // Direction offset for B channel
Motor motorFL = Motor(FAIN1, FAIN2, FPWMA, FOffsetA, FSTBY);
Motor motorFR = Motor(FBIN1, FBIN2, FPWMB, FOffsetB, FSTBY);
Motor motorRL = Motor(RBIN1, RBIN2, RPWMB, ROffsetB, RSTBY);
Motor motorRR = Motor(RAIN1, RAIN2, RPWMA, ROffsetA, RSTBY);

int FLEncoderPinA = 3;
int FLEncoderPinB = 14;
int FREncoderPinA = 2;
int FREncoderPinB = 15;
int RLEncoderPinA = 18;
int RLEncoderPinB = 16;
int RREncoderPinA = 19;
int RREncoderPinB = 17;

int FLEncoderALast = 0;
int FREncoderALast = 0;
int RLEncoderALast = 0;
int RREncoderALast = 0;

int FLEncoderTicks = 0; // Number of ticks
int FREncoderTicks = 0;// Number of ticks
int RLEncoderTicks = 0; // Number of ticks
int RREncoderTicks = 0;// Number of ticks

boolean FLEncoderDirxn = true; // Rotation direction
boolean FREncoderDirxn = true; // Rotation direction
boolean RLEncoderDirxn = true; // Rotation direction
boolean RREncoderDirxn = true; // Rotation direction

void setup()
{
  Serial.begin(115200); // Initialize the serial port
  encodersInit(); // Initialize the encoders
}

void loop()
{
  delay(10000); // Initialization delay

  // Drive forward
  for(int speedValue=0; speedValue<150; speedValue++){
    forward(motorFL, motorFR, speedValue);
    forward(motorRL, motorRR, speedValue);
    Serial.print("FL-Ticks:"); Serial.print(FLEncoderTicks); Serial.print(","); Serial.print("FR-Ticks:"); Serial.print(FREncoderTicks); Serial.print(",");
    Serial.print("RL-Ticks:"); Serial.print(RLEncoderTicks); Serial.print(","); Serial.print("RR-Ticks:"); Serial.println(RREncoderTicks);
    delay(25);
  }

  // Brake
  brake(motorFL, motorFR);
  brake(motorRL, motorRR);
  Serial.print("FL-Ticks:"); Serial.print(FLEncoderTicks); Serial.print(","); Serial.print("FR-Ticks:"); Serial.print(FREncoderTicks); Serial.print(",");
  Serial.print("RL-Ticks:"); Serial.print(RLEncoderTicks); Serial.print(","); Serial.print("RR-Ticks:"); Serial.println(RREncoderTicks);

  // Drive reverse
  for(int speedValue=0; speedValue<150; speedValue++){
    forward(motorFL, motorFR, -speedValue);
    forward(motorRL, motorRR, -speedValue);
    Serial.print("FL-Ticks:"); Serial.print(FLEncoderTicks); Serial.print(","); Serial.print("FR-Ticks:"); Serial.print(FREncoderTicks); Serial.print(",");
    Serial.print("RL-Ticks:"); Serial.print(RLEncoderTicks); Serial.print(","); Serial.print("RR-Ticks:"); Serial.println(RREncoderTicks);
    delay(25);
  }

  // Brake
  brake(motorFL, motorFR);
  brake(motorRL, motorRR);
  Serial.print("FL-Ticks:"); Serial.print(FLEncoderTicks); Serial.print(","); Serial.print("FR-Ticks:"); Serial.print(FREncoderTicks); Serial.print(",");
  Serial.print("RL-Ticks:"); Serial.print(RLEncoderTicks); Serial.print(","); Serial.print("RR-Ticks:"); Serial.println(RREncoderTicks);
}

void encodersInit()
{
  FLEncoderDirxn = true; // Default rotation direction: forward
  FREncoderDirxn = true; // Default rotation direction: forward
  RLEncoderDirxn = true; // Default rotation direction: forward
  RREncoderDirxn = true; // Default rotation direction: forward
  pinMode(FLEncoderPinB, INPUT); // Channel B
  pinMode(FREncoderPinB, INPUT); // Channel B
  pinMode(RLEncoderPinB, INPUT); // Channel B
  pinMode(RREncoderPinB, INPUT); // Channel B
  pinMode(FLEncoderPinA, INPUT_PULLUP); // Channel A
  pinMode(FREncoderPinA, INPUT_PULLUP); // Channel A
  pinMode(RLEncoderPinA, INPUT_PULLUP); // Channel A
  pinMode(RREncoderPinA, INPUT_PULLUP); // Channel A
  attachInterrupt(digitalPinToInterrupt(FLEncoderPinA), FLEncoderISR, CHANGE); // Declare interrupt
  attachInterrupt(digitalPinToInterrupt(FREncoderPinA), FREncoderISR, CHANGE); // Declare interrupt
  attachInterrupt(digitalPinToInterrupt(RLEncoderPinA), RLEncoderISR, CHANGE); // Declare interrupt
  attachInterrupt(digitalPinToInterrupt(RREncoderPinA), RREncoderISR, CHANGE); // Declare interrupt
}

void FLEncoderISR()
{
  int FLEncoderA = digitalRead(FLEncoderPinA);
  if((FLEncoderALast == LOW) && FLEncoderA == HIGH)
  {
    int FLEncoderB = digitalRead(FLEncoderPinB);
    if(FLEncoderB == LOW && FLEncoderDirxn)
    {
      FLEncoderDirxn = false; // Reverse
    }
    else if(FLEncoderB == HIGH && !FLEncoderDirxn)
    {
      FLEncoderDirxn = true;  // Forward
    }
  }
  FLEncoderALast = FLEncoderA;

  if(FLEncoderDirxn)  FLEncoderTicks--; // Change increment/decrement to match dirxn with other encoders
  else  FLEncoderTicks++;  // Change increment/decrement to match dirxn with other encoders
}

void FREncoderISR()
{
  int FREncoderA = digitalRead(FREncoderPinA);
  if((FREncoderALast == LOW) && FREncoderA == HIGH)
  {
    int FREncoderB = digitalRead(FREncoderPinB);
    if(FREncoderB == LOW && FREncoderDirxn)
    {
      FREncoderDirxn = false; // Reverse
    }
    else if(FREncoderB == HIGH && !FREncoderDirxn)
    {
      FREncoderDirxn = true;  // Forward
    }
  }
  FREncoderALast = FREncoderA;

  if(FREncoderDirxn)  FREncoderTicks++; // Change increment/decrement to match dirxn with other encoders
  else  FREncoderTicks--; // Change increment/decrement to match dirxn with other encoders
}

void RLEncoderISR()
{
  int RLEncoderA = digitalRead(RLEncoderPinA);
  if((RLEncoderALast == LOW) && RLEncoderA == HIGH)
  {
    int RLEncoderB = digitalRead(RLEncoderPinB);
    if(RLEncoderB == LOW && RLEncoderDirxn)
    {
      RLEncoderDirxn = false; // Reverse
    }
    else if(RLEncoderB == HIGH && !RLEncoderDirxn)
    {
      RLEncoderDirxn = true;  // Forward
    }
  }
  RLEncoderALast = RLEncoderA;

  if(RLEncoderDirxn)  RLEncoderTicks--; // Change increment/decrement to match dirxn with other encoders
  else  RLEncoderTicks++; // Change increment/decrement to match dirxn with other encoders
}

void RREncoderISR()
{
  int RREncoderA = digitalRead(RREncoderPinA);
  if((RREncoderALast == LOW) && RREncoderA == HIGH)
  {
    int RREncoderB = digitalRead(RREncoderPinB);
    if(RREncoderB == LOW && RREncoderDirxn)
    {
      RREncoderDirxn = false; // Reverse
    }
    else if(RREncoderB == HIGH && !RREncoderDirxn)
    {
      RREncoderDirxn = true;  // Forward
    }
  }
  RREncoderALast = RREncoderA;

  if(RREncoderDirxn)  RREncoderTicks--; // Change increment/decrement to match dirxn with other encoders
  else  RREncoderTicks++; // Change increment/decrement to match dirxn with other encoders
}
