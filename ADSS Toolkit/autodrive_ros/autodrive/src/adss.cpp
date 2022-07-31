#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>

void callback_throttle(const std_msgs::Float32::ConstPtr& msg){
  ROS_INFO("V1 Throttle: %f", msg->data);
}

void callback_steering(const std_msgs::Float32::ConstPtr& msg){
  ROS_INFO("V1 Steering: %f", msg->data);
}

void callback_left_encoder(const sensor_msgs::JointState::ConstPtr& msg){
  ROS_INFO("V1 Left Encoder Angle: %f", msg->position[0]);
}

void callback_right_encoder(const sensor_msgs::JointState::ConstPtr& msg){
  ROS_INFO("V1 Right Encoder Angle: %f", msg->position[1]);
}

void callback_ips(const geometry_msgs::Point::ConstPtr& msg){
  ROS_INFO("V1 Position: %f %f %f", msg->x, msg->y, msg->z);
}

void callback_imu(const sensor_msgs::Imu::ConstPtr& msg){
  ROS_INFO("V1 Orientation: %f %f %f %f", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  ROS_INFO("V1 Angular Velocity: %f %f %f", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  ROS_INFO("V1 Linear Acceleration: %f %f %f", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
}

void callback_lidar(const sensor_msgs::LaserScan::ConstPtr& msg){
  ROS_INFO("V1 LIDAR: Range Measurement: %f", msg->ranges[0]);
}

void callback_front_camera(const sensor_msgs::Image::ConstPtr& msg){
  ROS_INFO("V1 Front Camera: Image Data Received");
}

void callback_rear_camera(const sensor_msgs::Image::ConstPtr& msg){
  ROS_INFO("V1 Rear Camera: Image Data Received");
}

int main(int argc, char** argv){
  // Initialize ROS node
  ros::init(argc, argv, "autodrive_adss");

  // ROS node handle
  ros::NodeHandle ros_node_handle;

  // Subscribers
  ros::Subscriber sub_throttle = ros_node_handle.subscribe("/autodrive/v1/throttle", 1, callback_throttle);
  ros::Subscriber sub_steering = ros_node_handle.subscribe("/autodrive/v1/steering", 1, callback_steering);
  ros::Subscriber sub_left_encoder = ros_node_handle.subscribe("/autodrive/v1/left_encoder", 1, callback_left_encoder);
  ros::Subscriber sub_right_encoder = ros_node_handle.subscribe("/autodrive/v1/right_encoder", 1, callback_right_encoder);
  ros::Subscriber sub_ips = ros_node_handle.subscribe("/autodrive/v1/ips", 1, callback_ips);
  ros::Subscriber sub_imu = ros_node_handle.subscribe("/autodrive/v1/imu", 1, callback_imu);
  ros::Subscriber sub_lidar = ros_node_handle.subscribe("/autodrive/v1/lidar", 1, callback_lidar);
  ros::Subscriber sub_front_camera = ros_node_handle.subscribe("/autodrive/v1/front_camera", 1, callback_front_camera);
  ros::Subscriber sub_rear_camera = ros_node_handle.subscribe("/autodrive/v1/rear_camera", 1, callback_rear_camera);

  // Publishers
  ros::Publisher pub_throttle_command = ros_node_handle.advertise<std_msgs::Float32>("/autodrive/v1/throttle_command", 1);
  ros::Publisher pub_steering_command = ros_node_handle.advertise<std_msgs::Float32>("/autodrive/v1/steering_command", 1);
  ros::Publisher pub_headlights_command = ros_node_handle.advertise<std_msgs::Int32>("/autodrive/v1/headlights_command", 1);
  ros::Publisher pub_indicators_command = ros_node_handle.advertise<std_msgs::Int32>("/autodrive/v1/indicators_command", 1);

  // Control commands
  float throttle_command = 1; // [-1, 1]
  float steering_command = 1; // [-1, 1]
  int headlights_command = 1; // [0 = disabled, 1 = low beam, 2 = high beam]
  int indicators_command = 3; // [0 = disabled, 1 = left turn indicator, 2 = right turn indicator, 3 = hazard indicator]

  ros::Rate loop_rate(10); // Loop rate (Hz)

  while(ros::ok()){
    // Populate the messages
    std_msgs::Float32 throttle_cmd_msg; // Message instance for throttle command
    std_msgs::Float32 steering_cmd_msg; // Message instance for steering command
    std_msgs::Int32 headlights_cmd_msg; // Message instance for headlights command
    std_msgs::Int32 indicators_cmd_msg; // Message instance for indicators command
    // Message data
    throttle_cmd_msg.data = throttle_command;
    steering_cmd_msg.data = steering_command;
    headlights_cmd_msg.data = headlights_command;
    indicators_cmd_msg.data = indicators_command;
    // Publish control commands
    pub_throttle_command.publish(throttle_cmd_msg);
    pub_steering_command.publish(steering_cmd_msg);
    pub_headlights_command.publish(headlights_cmd_msg);
    pub_indicators_command.publish(indicators_cmd_msg);
    // Spin once to allow calling callbacks
    ros::spinOnce();
    // Maintain loop rate
    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}
