#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <Servo.h>

ros::NodeHandle ROS;

Servo steeringServo;
int servoPin = 9;
int zeroSteerAngle = 97; // Calibrated zero-steer angle
int steering = zeroSteerAngle;

int speedPin = 6;
int dirxnPin = 7;
int brakePin = 8;
int throttle = 0;

void steer_init()
{
  steeringServo.write(zeroSteerAngle); // Initialize servo on power-up to zero-steer angle
  steeringServo.attach(servoPin); // Initialize the steering actuator
}

void drive_init()
{ 
  pinMode(speedPin, OUTPUT); // Declare speed pin
  pinMode(brakePin, OUTPUT); // Declare brake pin
  pinMode(dirxnPin, OUTPUT); // Declare direction pin
}

void steer_cb(const std_msgs::Float32& steer_msg)
{
  steering = zeroSteerAngle + 30*(steer_msg.data);
  steeringServo.write(steering);
}

void drive_cb(const std_msgs::Float32& drive_msg)
{
  throttle = 255*drive_msg.data;
  // Drive forward
  if (drive_msg.data > 0)
  {
    digitalWrite(dirxnPin, 0);
    digitalWrite(brakePin, 0);
    analogWrite(speedPin, throttle);
  }
  // Drive reverse
  else if (drive_msg.data < 0)
  {
    digitalWrite(dirxnPin, 1);
    digitalWrite(brakePin, 0);
    analogWrite(speedPin, -throttle);
  }
  // Brake
  else
  {
    analogWrite(speedPin, 0);
    digitalWrite(brakePin, 1);
  }
}

ros::Subscriber<std_msgs::Float32> steer_sub("steer_cmd", steer_cb);
ros::Subscriber<std_msgs::Float32> drive_sub("drive_cmd", drive_cb);

void setup()
{
  steer_init(); // Initialize steering actuator
  drive_init(); // Initialize drive actuators

  ROS.initNode();
  ROS.subscribe(steer_sub);
  ROS.subscribe(drive_sub);
}

void loop()
{
  ROS.spinOnce();
  delay(1);
}
