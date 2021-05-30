#include <Servo.h>
int servoPin = 9;
Servo steeringServo;
int zeroSteerAngle = 97; // Calibrated zero-steer angle
float steeringAngle = 0;

void setup()
{
  Serial.begin(115200); // Initialize the serial port
  steeringServo.write(zeroSteerAngle); // Initialize servo on power-up to zero-steer angle
  steeringServo.attach(servoPin); // Initialize the steering actuator
}

void loop()
{
  delay(10000); // Initialization delay

  for(int i=0; i<=3; i++){
    for(steeringAngle=zeroSteerAngle; steeringAngle>=zeroSteerAngle-30; steeringAngle--){
      steeringServo.write(steeringAngle);
      Serial.print("Steering:"); Serial.println((steeringAngle-zeroSteerAngle)/30);
      delay(25);
    }
  
    for(steeringAngle=zeroSteerAngle-30; steeringAngle<=zeroSteerAngle+30; steeringAngle++){
      steeringServo.write(steeringAngle);
      Serial.print("Steering:"); Serial.println((steeringAngle-zeroSteerAngle)/30);
      delay(25);
    }
  
    for(steeringAngle=zeroSteerAngle+30; steeringAngle>=zeroSteerAngle; steeringAngle--){
      steeringServo.write(steeringAngle);
      Serial.print("Steering:"); Serial.println((steeringAngle-zeroSteerAngle)/30);
      delay(25);
    }
  }
}
