/////////////////////////////////////////////////////////////////////////////////

// Copyright (c) 2023, Tinker Twins
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <std_msgs/Int32.h>

void callback_signal_1_state(const std_msgs::Int32::ConstPtr& msg){
  ROS_INFO("TL1 State: %d", msg->data);
}

void callback_signal_2_state(const std_msgs::Int32::ConstPtr& msg){
  ROS_INFO("TL2 State: %d", msg->data);
}

void callback_signal_3_state(const std_msgs::Int32::ConstPtr& msg){
  ROS_INFO("TL3 State: %d", msg->data);
}

void callback_signal_4_state(const std_msgs::Int32::ConstPtr& msg){
  ROS_INFO("TL4 State: %d", msg->data);
}

int main(int argc, char** argv){
  // Initialize ROS node
  ros::init(argc, argv, "autodrive_scss");

  // ROS node handle
  ros::NodeHandle ros_node_handle;

  // Subscribers
  ros::Subscriber sub_signal_1_state = ros_node_handle.subscribe("/autodrive/signal_1/state", 1, callback_signal_1_state);
  ros::Subscriber sub_signal_2_state = ros_node_handle.subscribe("/autodrive/signal_2/state", 1, callback_signal_2_state);
  ros::Subscriber sub_signal_3_state = ros_node_handle.subscribe("/autodrive/signal_3/state", 1, callback_signal_3_state);
  ros::Subscriber sub_signal_4_state = ros_node_handle.subscribe("/autodrive/signal_4/state", 1, callback_signal_4_state);

  // Publishers
  ros::Publisher pub_signal_1_command = ros_node_handle.advertise<std_msgs::Int32>("/autodrive/signal_1/command", 1);
  ros::Publisher pub_signal_2_command = ros_node_handle.advertise<std_msgs::Int32>("/autodrive/signal_2/command", 1);
  ros::Publisher pub_signal_3_command = ros_node_handle.advertise<std_msgs::Int32>("/autodrive/signal_3/command", 1);
  ros::Publisher pub_signal_4_command = ros_node_handle.advertise<std_msgs::Int32>("/autodrive/signal_4/command", 1);

  // Control commands
  int signal_1_command = 1; // [0 = disabled, 1 = red, 2 = yellow, 3 = green]
  int signal_2_command = 2; // [0 = disabled, 1 = red, 2 = yellow, 3 = green]
  int signal_3_command = 3; // [0 = disabled, 1 = red, 2 = yellow, 3 = green]
  int signal_4_command = 3; // [0 = disabled, 1 = red, 2 = yellow, 3 = green]

  ros::Rate loop_rate(10); // Loop rate (Hz)

  while(ros::ok()){
    // Populate the messages
    std_msgs::Int32 signal_1_cmd_msg; // Message instance for TL1 command
    std_msgs::Int32 signal_2_cmd_msg; // Message instance for TL2 command
    std_msgs::Int32 signal_3_cmd_msg; // Message instance for TL3 command
    std_msgs::Int32 signal_4_cmd_msg; // Message instance for TL4 command
    // Message data
    signal_1_cmd_msg.data = signal_1_command;
    signal_2_cmd_msg.data = signal_2_command;
    signal_3_cmd_msg.data = signal_3_command;
    signal_4_cmd_msg.data = signal_4_command;
    // Publish control commands
    pub_signal_1_command.publish(signal_1_cmd_msg);
    pub_signal_2_command.publish(signal_2_cmd_msg);
    pub_signal_3_command.publish(signal_3_cmd_msg);
    pub_signal_4_command.publish(signal_4_cmd_msg);
    // Spin once to allow calling callbacks
    ros::spinOnce();
    // Maintain loop rate
    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}
