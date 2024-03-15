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

int PWMValue = 0;
float throttle = 0.0;

void setup()
{
  Serial.begin(115200); // Initialize the serial port
}


void loop()
{
  delay(10000); // Initialization delay

  // Drive forward
  for(PWMValue=0; PWMValue<=256; PWMValue++){
    forward(motorFL, motorFR, PWMValue);
    forward(motorRL, motorRR, PWMValue);
    throttle += 0.390625;
    Serial.print("FL-Throttle:"); Serial.print(throttle); Serial.print(","); Serial.print("FR-Throttle:"); Serial.print(throttle); Serial.print(",");
    Serial.print("RL-Throttle:"); Serial.print(throttle); Serial.print(","); Serial.print("RR-Throttle:"); Serial.println(throttle);
    delay(25);
  }

  // Brake
  brake(motorFL, motorFR);
  brake(motorRL, motorRR);
  throttle = 0;
  Serial.print("FL-Throttle:"); Serial.print(throttle); Serial.print(","); Serial.print("FR-Throttle:"); Serial.print(throttle); Serial.print(",");
  Serial.print("RL-Throttle:"); Serial.print(throttle); Serial.print(","); Serial.print("RR-Throttle:"); Serial.println(throttle);

  // Drive reverse
  for(PWMValue=0; PWMValue<=256; PWMValue++){
    forward(motorFL, motorFR, -PWMValue);
    forward(motorRL, motorRR, -PWMValue);
    throttle -= 0.390625;
    Serial.print("FL-Throttle:"); Serial.print(throttle); Serial.print(","); Serial.print("FR-Throttle:"); Serial.print(throttle); Serial.print(",");
    Serial.print("RL-Throttle:"); Serial.print(throttle); Serial.print(","); Serial.print("RR-Throttle:"); Serial.println(throttle);
    delay(25);
  }

  // Brake
  brake(motorFL, motorFR);
  brake(motorRL, motorRR);
  throttle = 0;
  Serial.print("FL-Throttle:"); Serial.print(throttle); Serial.print(","); Serial.print("FR-Throttle:"); Serial.print(throttle); Serial.print(",");
  Serial.print("RL-Throttle:"); Serial.print(throttle); Serial.print(","); Serial.print("RR-Throttle:"); Serial.println(throttle);
}
