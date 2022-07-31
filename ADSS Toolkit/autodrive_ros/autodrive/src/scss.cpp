#include <ros/ros.h>
#include <std_msgs/Int32.h>

void callback_tl1_state(const std_msgs::Int32::ConstPtr& msg){
  ROS_INFO("TL1 State: %d", msg->data);
}

void callback_tl2_state(const std_msgs::Int32::ConstPtr& msg){
  ROS_INFO("TL2 State: %d", msg->data);
}

void callback_tl3_state(const std_msgs::Int32::ConstPtr& msg){
  ROS_INFO("TL3 State: %d", msg->data);
}

void callback_tl4_state(const std_msgs::Int32::ConstPtr& msg){
  ROS_INFO("TL4 State: %d", msg->data);
}

int main(int argc, char** argv){
  // Initialize ROS node
  ros::init(argc, argv, "autodrive_scss");

  // ROS node handle
  ros::NodeHandle ros_node_handle;

  // Subscribers
  ros::Subscriber sub_tl1_state = ros_node_handle.subscribe("/autodrive/tl1/state", 1, callback_tl1_state);
  ros::Subscriber sub_tl2_state = ros_node_handle.subscribe("/autodrive/tl2/state", 1, callback_tl2_state);
  ros::Subscriber sub_tl3_state = ros_node_handle.subscribe("/autodrive/tl3/state", 1, callback_tl3_state);
  ros::Subscriber sub_tl4_state = ros_node_handle.subscribe("/autodrive/tl4/state", 1, callback_tl4_state);

  // Publishers
  ros::Publisher pub_tl1_command = ros_node_handle.advertise<std_msgs::Int32>("/autodrive/tl1/command", 1);
  ros::Publisher pub_tl2_command = ros_node_handle.advertise<std_msgs::Int32>("/autodrive/tl2/command", 1);
  ros::Publisher pub_tl3_command = ros_node_handle.advertise<std_msgs::Int32>("/autodrive/tl3/command", 1);
  ros::Publisher pub_tl4_command = ros_node_handle.advertise<std_msgs::Int32>("/autodrive/tl4/command", 1);

  // Control commands
  int tl1_command = 1; // [0 = disabled, 1 = red, 2 = yellow, 3 = green]
  int tl2_command = 2; // [0 = disabled, 1 = red, 2 = yellow, 3 = green]
  int tl3_command = 3; // [0 = disabled, 1 = red, 2 = yellow, 3 = green]
  int tl4_command = 3; // [0 = disabled, 1 = red, 2 = yellow, 3 = green]

  ros::Rate loop_rate(10); // Loop rate (Hz)

  while(ros::ok()){
    // Populate the messages
    std_msgs::Int32 tl1_cmd_msg; // Message instance for TL1 command
    std_msgs::Int32 tl2_cmd_msg; // Message instance for TL2 command
    std_msgs::Int32 tl3_cmd_msg; // Message instance for TL3 command
    std_msgs::Int32 tl4_cmd_msg; // Message instance for TL4 command
    // Message data
    tl1_cmd_msg.data = tl1_command;
    tl2_cmd_msg.data = tl2_command;
    tl3_cmd_msg.data = tl3_command;
    tl4_cmd_msg.data = tl4_command;
    // Publish control commands
    pub_tl1_command.publish(tl1_cmd_msg);
    pub_tl2_command.publish(tl2_cmd_msg);
    pub_tl3_command.publish(tl3_cmd_msg);
    pub_tl4_command.publish(tl4_cmd_msg);
    // Spin once to allow calling callbacks
    ros::spinOnce();
    // Maintain loop rate
    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}
