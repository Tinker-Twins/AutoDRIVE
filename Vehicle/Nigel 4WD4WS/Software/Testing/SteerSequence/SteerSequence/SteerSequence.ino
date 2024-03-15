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
  // Initialization
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  delay(5000);

  // Front steering
  servoFL.write(zeroSteerFL+30);
  servoFR.write(zeroSteerFR+30);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  delay(1000);
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  delay(1000);
  servoFL.write(zeroSteerFL-30);
  servoFR.write(zeroSteerFR-30);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  delay(1000);

  // Transition delay
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  delay(2000);

  // Rear steering
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL+30);
  servoRR.write(zeroSteerRR+30);
  delay(1000);
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  delay(1000);
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL-30);
  servoRR.write(zeroSteerRR-30);
  delay(1000);

  // Transition delay
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  delay(2000);

  // Out-of-phase steering
  servoFL.write(zeroSteerFL+20);
  servoFR.write(zeroSteerFR+20);
  servoRL.write(zeroSteerRL-20);
  servoRR.write(zeroSteerRR-20);
  delay(1000);
  servoFL.write(zeroSteerFL+40);
  servoFR.write(zeroSteerFR+40);
  servoRL.write(zeroSteerRL-40);
  servoRR.write(zeroSteerRR-40);
  delay(1000);
  servoFL.write(zeroSteerFL+20);
  servoFR.write(zeroSteerFR+20);
  servoRL.write(zeroSteerRL-20);
  servoRR.write(zeroSteerRR-20);
  delay(1000);
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  delay(1000);
  servoFL.write(zeroSteerFL-20);
  servoFR.write(zeroSteerFR-20);
  servoRL.write(zeroSteerRL+20);
  servoRR.write(zeroSteerRR+20);
  delay(1000);
  servoFL.write(zeroSteerFL-40);
  servoFR.write(zeroSteerFR-40);
  servoRL.write(zeroSteerRL+40);
  servoRR.write(zeroSteerRR+40);
  delay(1000);
  servoFL.write(zeroSteerFL-20);
  servoFR.write(zeroSteerFR-20);
  servoRL.write(zeroSteerRL+20);
  servoRR.write(zeroSteerRR+20);
  delay(1000);
  
  // Transition delay
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  delay(2000);

  // In-phase steering
  servoFL.write(zeroSteerFL+20);
  servoFR.write(zeroSteerFR+20);
  servoRL.write(zeroSteerRL+20);
  servoRR.write(zeroSteerRR+20);
  delay(1000);
  servoFL.write(zeroSteerFL+40);
  servoFR.write(zeroSteerFR+40);
  servoRL.write(zeroSteerRL+40);
  servoRR.write(zeroSteerRR+40);
  delay(1000);
  servoFL.write(zeroSteerFL+20);
  servoFR.write(zeroSteerFR+20);
  servoRL.write(zeroSteerRL+20);
  servoRR.write(zeroSteerRR+20);
  delay(1000);
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  delay(1000);
  servoFL.write(zeroSteerFL-20);
  servoFR.write(zeroSteerFR-20);
  servoRL.write(zeroSteerRL-20);
  servoRR.write(zeroSteerRR-20);
  delay(1000);
  servoFL.write(zeroSteerFL-40);
  servoFR.write(zeroSteerFR-40);
  servoRL.write(zeroSteerRL-40);
  servoRR.write(zeroSteerRR-40);
  delay(1000);
  servoFL.write(zeroSteerFL-20);
  servoFR.write(zeroSteerFR-20);
  servoRL.write(zeroSteerRL-20);
  servoRR.write(zeroSteerRR-20);
  delay(1000);
  
  // Transition delay
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  delay(2000);

  // Oblique steering
  servoFL.write(zeroSteerFL+22.5);
  servoFR.write(zeroSteerFR-22.5);
  servoRL.write(zeroSteerRL-22.5);
  servoRR.write(zeroSteerRR+22.5);
  delay(1000);
  servoFL.write(zeroSteerFL+45);
  servoFR.write(zeroSteerFR-45);
  servoRL.write(zeroSteerRL-45);
  servoRR.write(zeroSteerRR+45);
  delay(3000);
  servoFL.write(zeroSteerFL+22.5);
  servoFR.write(zeroSteerFR-22.5);
  servoRL.write(zeroSteerRL-22.5);
  servoRR.write(zeroSteerRR+22.5);
  delay(1000);

  // Transition delay
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  delay(2000);

  // Crab-walk steering
  servoFL.write(zeroSteerFL-45);
  servoFR.write(zeroSteerFR+45);
  servoRL.write(zeroSteerRL+45);
  servoRR.write(zeroSteerRR-45);
  delay(1000);
  servoFL.write(zeroSteerFL-90);
  servoFR.write(zeroSteerFR+90);
  servoRL.write(zeroSteerRL+90);
  servoRR.write(zeroSteerRR-90);
  delay(3000);
  servoFL.write(zeroSteerFL-45);
  servoFR.write(zeroSteerFR+45);
  servoRL.write(zeroSteerRL+45);
  servoRR.write(zeroSteerRR-45);
  delay(1000);
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  delay(1000);
}
