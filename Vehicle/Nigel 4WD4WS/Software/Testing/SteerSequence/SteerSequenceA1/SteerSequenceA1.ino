#include <Servo.h>

int servoLeftPin = 5;
int servoRightPin = 6;
Servo servoLeft;
Servo servoRight;
int zeroSteerLeft = 90; // Calibrated zero-steer angle
int zeroSteerRight = 90; // Calibrated zero-steer angle

void setup()
{
  servoLeft.write(zeroSteerLeft); // Initialize servo on power-up to zero-steer angle
  servoRight.write(zeroSteerRight); // Initialize servo on power-up to zero-steer angle
  servoLeft.attach(servoLeftPin); // Initialize the steering actuator
  servoRight.attach(servoRightPin); // Initialize the steering actuator
}

void loop()
{
  // Initialization
  servoLeft.write(zeroSteerLeft);
  servoRight.write(zeroSteerRight);
  delay(4000);

  // Front steering
  servoLeft.write(zeroSteerLeft);
  servoRight.write(zeroSteerRight);
  delay(6000);

  // Rear steering
  servoLeft.write(zeroSteerLeft+30);
  servoRight.write(zeroSteerRight+30);
  delay(1000);
  servoLeft.write(zeroSteerLeft);
  servoRight.write(zeroSteerRight);
  delay(1000);
  servoLeft.write(zeroSteerLeft-30);
  servoRight.write(zeroSteerRight-30);
  delay(1000);

  servoLeft.write(zeroSteerLeft);
  servoRight.write(zeroSteerRight);
  delay(2000);  

  // Out-of-phase steering
  servoLeft.write(zeroSteerLeft-20);
  servoRight.write(zeroSteerRight-20);
  delay(1000);
  servoLeft.write(zeroSteerLeft-40);
  servoRight.write(zeroSteerRight-40);
  delay(1000);
  servoLeft.write(zeroSteerLeft-20);
  servoRight.write(zeroSteerRight-20);
  delay(1000);
  servoLeft.write(zeroSteerLeft);
  servoRight.write(zeroSteerRight);
  delay(1000);
  servoLeft.write(zeroSteerLeft+20);
  servoRight.write(zeroSteerRight+20);
  delay(1000);
  servoLeft.write(zeroSteerLeft+40);
  servoRight.write(zeroSteerRight+40);
  delay(1000);
  servoLeft.write(zeroSteerLeft+20);
  servoRight.write(zeroSteerRight+20);
  delay(1000);
  
  servoLeft.write(zeroSteerLeft);
  servoRight.write(zeroSteerRight);
  delay(2000);

  // In-phase steering
  servoLeft.write(zeroSteerLeft+20);
  servoRight.write(zeroSteerRight+20);
  delay(1000);
  servoLeft.write(zeroSteerLeft+40);
  servoRight.write(zeroSteerRight+40);
  delay(1000);
  servoLeft.write(zeroSteerLeft+20);
  servoRight.write(zeroSteerRight+20);
  delay(1000);
  servoLeft.write(zeroSteerLeft);
  servoRight.write(zeroSteerRight);
  delay(1000);
  servoLeft.write(zeroSteerLeft-20);
  servoRight.write(zeroSteerRight-20);
  delay(1000);
  servoLeft.write(zeroSteerLeft-40);
  servoRight.write(zeroSteerRight-40);
  delay(1000);
  servoLeft.write(zeroSteerLeft-20);
  servoRight.write(zeroSteerRight-20);
  delay(1000);
  
  servoLeft.write(zeroSteerLeft);
  servoRight.write(zeroSteerRight);
  delay(2000);

  // Oblique steering
  servoLeft.write(zeroSteerLeft-22.5);
  servoRight.write(zeroSteerRight+22.5);
  delay(1000);
  servoLeft.write(zeroSteerLeft-45);
  servoRight.write(zeroSteerRight+45);
  delay(3000);
  servoLeft.write(zeroSteerLeft-22.5);
  servoRight.write(zeroSteerRight+22.5);
  delay(1000);

  servoLeft.write(zeroSteerLeft);
  servoRight.write(zeroSteerRight);
  delay(2000);

  // Crab-walk steering
  servoLeft.write(zeroSteerLeft+45);
  servoRight.write(zeroSteerRight-45);
  delay(1000);
  servoLeft.write(zeroSteerLeft+90);
  servoRight.write(zeroSteerRight-90);
  delay(3000);
  servoLeft.write(zeroSteerLeft+45);
  servoRight.write(zeroSteerRight-45);
  delay(1000);
  servoLeft.write(zeroSteerLeft);
  servoRight.write(zeroSteerRight);
  delay(1000);
}