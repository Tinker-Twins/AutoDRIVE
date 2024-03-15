#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <SparkFun_TB6612.h>
#include <Servo.h>

ros::NodeHandle ROS;

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

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

int speedValue = 0; // Commanded PWM value based on throttle %

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

int steeringAngle=0; // Commanded steering angle
int steeringAngleL=0; // Left-wheel steering angle
int steeringAngleR=0; // Right-wheel steering angle

float Wheelbase = 141.54; // mm
float TrackWidth = 153.00; // mm

void steer_cb(const std_msgs::Float32& steer_msg)
{
  steeringAngle = 30*(steer_msg.data);
  steeringAngleL = RAD_TO_DEG*atan((2*Wheelbase*tan(DEG_TO_RAD*steeringAngle))/((2*Wheelbase)-(TrackWidth*tan(DEG_TO_RAD*steeringAngle))));
  steeringAngleR = RAD_TO_DEG*atan((2*Wheelbase*tan(DEG_TO_RAD*steeringAngle))/((2*Wheelbase)+(TrackWidth*tan(DEG_TO_RAD*steeringAngle))));
  servoFL.write(zeroSteerFL-steeringAngleL);
  servoFR.write(zeroSteerFR-steeringAngleR);
  servoRL.write(zeroSteerRL);
  servoRR.write(zeroSteerRR);
}

void drive_cb(const std_msgs::Float32& drive_msg)
{
  speedValue = 255*drive_msg.data;
  // Drive forward
  if (drive_msg.data > 0)
  {
    forward(motorFL, motorFR, speedValue);
    forward(motorRL, motorRR, speedValue);
  }
  // Drive reverse
  else if (drive_msg.data < 0)
  {
    back(motorFL, motorFR, speedValue);
    back(motorRL, motorRR, speedValue);
  }
  // Brake
  else
  {
    brake(motorFL, motorFR);
    brake(motorRL, motorRR);
  }
}

ros::Subscriber<std_msgs::Float32> steer_sub("autodrive/nigel_1/steering_command", steer_cb);
ros::Subscriber<std_msgs::Float32> drive_sub("autodrive/nigel_1/throttle_command", drive_cb);

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
  
  Serial.begin(115200);
  ROS.getHardware()->setBaud(115200);
  ROS.initNode();
  ROS.subscribe(steer_sub);
  ROS.subscribe(drive_sub);
}

void loop()
{
  ROS.spinOnce();
  delay(1);
}
