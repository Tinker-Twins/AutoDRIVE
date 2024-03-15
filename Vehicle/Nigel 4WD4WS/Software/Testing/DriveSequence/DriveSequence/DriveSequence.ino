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

void setup()
{
}


void loop()
{
  // Initialization
  brake(motorFL, motorFR);
  brake(motorRL, motorRR);
  delay(4000);

  // All-wheel drive
  forward(motorFL, motorFR, 40);
  forward(motorRL, motorRR, 40);
  delay(7000);

  // Front-wheel drive
  forward(motorFL, motorFR, 40);
  brake(motorRL, motorRR);
  delay(7000);

  // Rear-wheel drive
  brake(motorFL, motorFR);
  forward(motorRL, motorRR, 40);
  delay(7000);

  // Neutral-steer drive
  left(motorFL, motorFR, 80);
  left(motorRL, motorRR, 80);
  delay(7000);

  // Pivot-steer drive
  motorFL.brake();
  motorRL.brake();
  motorFR.drive(40, 7000);
  motorRR.drive(40, 7000);
  
  // Torque-vector drive
  motorFL.drive(40);
  motorFR.drive(20);
  motorRL.drive(40);
  motorRR.drive(20);
  delay(7000);
}