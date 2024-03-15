#include <SparkFun_TB6612.h>
#include <Servo.h>

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

int speedValue = 150;

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

int tailLights = 45;
int headLightsLB = 35;
int headLightsHB = 37;
int reverseIndicators = 43;
int leftIndicators = 39;
int rightIndicators = 41;

void setup()
{
  Serial.begin(115200); // Initialize the serial port
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
  // Initialize
  brake(motorFL, motorFR);
  brake(motorRL, motorRR);
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  analogWrite(tailLights, 255);
  delay(10000);

  // Move straight
  forward(motorFL, motorFR, 150);
  forward(motorRL, motorRR, 150);
  servoFL.write(zeroSteerFL-6);
  servoFR.write(zeroSteerFR-6);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  digitalWrite(headLightsLB, 1);
  digitalWrite(headLightsHB, 1);
  analogWrite(tailLights, 100);
  delay(3000);

  // Stop
  brake(motorFL, motorFR);
  brake(motorRL, motorRR);
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  digitalWrite(headLightsLB, 1);
  digitalWrite(headLightsHB, 1);
  analogWrite(tailLights, 255);
  delay(1000);

  // Parallel park using crab-walk steering
  forward(motorFR, motorRL, 100);
  forward(motorFL, motorRR, -100);
  servoFL.write(zeroSteerFL-90);
  servoFR.write(zeroSteerFR+90);
  servoRL.write(zeroSteerRL+90);
  servoRR.write(zeroSteerRR-90);
  digitalWrite(headLightsLB, 1);
  digitalWrite(headLightsHB, 1);
  analogWrite(tailLights, 100);
  delay(4000);

  // Parking delay
  brake(motorFL, motorFR);
  brake(motorRL, motorRR);
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  digitalWrite(headLightsLB, 0);
  digitalWrite(headLightsHB, 0);
  analogWrite(tailLights, 255);
  delay(5000);

  // Exit parking using crab-walk steering
  forward(motorFR, motorRL, -100);
  forward(motorFL, motorRR, 100);
  servoFL.write(zeroSteerFL-90);
  servoFR.write(zeroSteerFR+90);
  servoRL.write(zeroSteerRL+90);
  servoRR.write(zeroSteerRR-90);
  digitalWrite(headLightsLB, 1);
  digitalWrite(headLightsHB, 1);
  analogWrite(tailLights, 100);
  delay(3750);

  // Stop
  brake(motorFL, motorFR);
  brake(motorRL, motorRR);
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  digitalWrite(headLightsLB, 1);
  digitalWrite(headLightsHB, 1);
  analogWrite(tailLights, 255);
  delay(1000);

  // Rotate in-place using oblique steering
  forward(motorFL, motorRL, -100);
  forward(motorFR, motorRR, 100);
  servoFL.write(zeroSteerFL+45);
  servoFR.write(zeroSteerFR-45);
  servoRL.write(zeroSteerRL-45);
  servoRR.write(zeroSteerRR+45);
  digitalWrite(headLightsLB, 1);
  digitalWrite(headLightsHB, 1);
  analogWrite(tailLights, 100);
  delay(4000);

  // Move straight
  forward(motorFL, motorFR, 150);
  forward(motorRL, motorRR, 150);
  servoFL.write(zeroSteerFL-6);
  servoFR.write(zeroSteerFR-6);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  digitalWrite(headLightsLB, 1);
  digitalWrite(headLightsHB, 1);
  analogWrite(tailLights, 100);
  delay(5000);
}
