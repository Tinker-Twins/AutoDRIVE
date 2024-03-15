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
  servoFL.write(zeroSteerFL);
  servoFR.write(zeroSteerFR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
  delay(10000); // Initialization delay

  for(int i=0; i<=3; i++){
    for(int steeringAngle=0; steeringAngle>-90; steeringAngle--){
      servoFL.write(zeroSteerFL+steeringAngle);
      servoFR.write(zeroSteerFR+steeringAngle);
      servoRL.write(zeroSteerRL+steeringAngle);
      servoRR.write(zeroSteerRR+steeringAngle);
      Serial.print("FL-Steering:"); Serial.print(zeroSteerFL+steeringAngle); Serial.print(","); Serial.print("FR-Steering:"); Serial.print(zeroSteerFR+steeringAngle); Serial.print(",");
      Serial.print("RL-Steering:"); Serial.print(zeroSteerRL+steeringAngle); Serial.print(","); Serial.print("RR-Steering:"); Serial.println(zeroSteerRR+steeringAngle);
      delay(25);
    }
  
    for(int steeringAngle=-90; steeringAngle<=90; steeringAngle++){
      servoFL.write(zeroSteerFL+steeringAngle);
      servoFR.write(zeroSteerFR+steeringAngle);
      servoRL.write(zeroSteerRL+steeringAngle);
      servoRR.write(zeroSteerRR+steeringAngle);
      Serial.print("FL-Steering:"); Serial.print(zeroSteerFL+steeringAngle); Serial.print(","); Serial.print("FR-Steering:"); Serial.print(zeroSteerFR+steeringAngle); Serial.print(",");
      Serial.print("RL-Steering:"); Serial.print(zeroSteerRL+steeringAngle); Serial.print(","); Serial.print("RR-Steering:"); Serial.println(zeroSteerRR+steeringAngle);
      delay(25);
    }
  
    for(int steeringAngle=90; steeringAngle>=0; steeringAngle--){
      servoFL.write(zeroSteerFL+steeringAngle);
      servoFR.write(zeroSteerFR+steeringAngle);
      servoRL.write(zeroSteerRL+steeringAngle);
      servoRR.write(zeroSteerRR+steeringAngle);
      Serial.print("FL-Steering:"); Serial.print(zeroSteerFL+steeringAngle); Serial.print(","); Serial.print("FR-Steering:"); Serial.print(zeroSteerFR+steeringAngle); Serial.print(",");
      Serial.print("RL-Steering:"); Serial.print(zeroSteerRL+steeringAngle); Serial.print(","); Serial.print("RR-Steering:"); Serial.println(zeroSteerRR+steeringAngle);
      delay(25);
    }
  }
}
