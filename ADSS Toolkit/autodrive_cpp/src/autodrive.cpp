// Import library
#include "autodrive.hpp"

////////////////////////////////////////////////////////////////////////////////

// VEHICLE CLASS METHODS

// Parse vehicle sensor data
void Vehicle::parse_data(json data, bool verbose=false) {
  // Actuator feedbacks
  throttle_str = data[1][id + " Throttle"];
  throttle = stof(throttle_str);
  steering_str = data[1][id + " Steering"];
  steering = stof(steering_str);
  // Wheel encoders
  encoder_ticks_str = data[1][id + " Encoder Ticks"];
  encoder_ticks = string_to_array(encoder_ticks_str);
  encoder_angles_str = data[1][id + " Encoder Angles"];
  encoder_angles = string_to_array(encoder_angles_str);
  // IPS
  position_str = data[1][id + " Position"];
  position = string_to_array(position_str);
  // IMU
  orientation_quaternion_str = data[1][id + " Orientation Quaternion"];
  orientation_quaternion = string_to_array(orientation_quaternion_str);
  orientation_euler_angles_str = data[1][id + " Orientation Euler Angles"];
  orientation_euler_angles = string_to_array(orientation_euler_angles_str);
  angular_velocity_str = data[1][id + " Angular Velocity"];
  angular_velocity = string_to_array(angular_velocity_str);
  linear_acceleration_str = data[1][id + " Linear Acceleration"];
  linear_acceleration = string_to_array(linear_acceleration_str);
  // LIDAR
  lidar_scan_rate_str = data[1][id + " LIDAR Scan Rate"];
  lidar_scan_rate = stof(lidar_scan_rate_str);
  lidar_range_array_str = data[1][id + " LIDAR Range Array"];
  lidar_range_array = string_to_array(lidar_range_array_str);
  lidar_intensity_array_str = data[1][id + " LIDAR Intensity Array"];
  lidar_intensity_array = string_to_array(lidar_intensity_array_str);
  // Cameras
  front_camera_image_str = data[1][id + " Front Camera Image"];
  front_camera_image = string_to_image(front_camera_image_str);
  rear_camera_image_str = data[1][id + " Rear Camera Image"];
  rear_camera_image = string_to_image(rear_camera_image_str);
  if (verbose) {
    cout << "\n--------------------------------" << endl;
    cout << "Receive Data from Vehicle: " << id << endl;
    cout << "--------------------------------" << endl;
    // Monitor vehicle data
    cout << "Throttle: " << throttle_str << endl;
    cout << "Steering: " << steering_str << endl;
    cout << "Encoder Ticks: " << encoder_ticks_str << endl;
    cout << "Encoder Angles: " << encoder_angles_str << endl;
    cout << "Position: " << position_str << endl;
    cout << "Orientation [Quaternion]: " << orientation_quaternion_str << endl;
    cout << "Orientation [Euler Angles]: " << orientation_euler_angles_str << endl;
    cout << "Angular Velocity: " << angular_velocity_str << endl;
    cout << "Linear Acceleration: " << linear_acceleration_str << endl;
    cout << "LIDAR Scan Rate: " << lidar_scan_rate_str << endl;
    cout << "LIDAR Range Array: " << endl << lidar_range_array_str << endl;
    cout << "LIDAR Intensity Array: " << endl << lidar_intensity_array_str << endl;
    resize(front_camera_image, front_camera_image, Size(640, 360));
    resize(rear_camera_image, rear_camera_image, Size(640, 360));
    imshow(id + " Front Camera Preview", front_camera_image);
    imshow(id + " Rear Camera Preview", rear_camera_image);
    waitKey(1);
  }
}

// Generate vehicle control commands
json Vehicle::generate_commands(json json_msg, bool verbose=false) {
  if (verbose) {
    cout << "\n-------------------------------" << endl;
    cout << "Transmit Data to Vehicle: " << id << endl;
    cout << "-------------------------------" << endl;
    // Monitor vehicle control commands
    throttle_command_str = to_string(throttle_command);
    steering_command_str = to_string(steering_command);
    cout << "Throttle Command: " << throttle_command_str << endl;
    cout << "Steering Command: " << steering_command_str << endl;
    if (headlights_command == 0) {
      headlights_command_str = "Disabled";
    }
    else if (headlights_command == 1) {
      headlights_command_str = "Low Beam";
    }
    else if (headlights_command == 2) {
      headlights_command_str = "High Beam";
    }
    else {
      headlights_command_str = "Invalid";
    }
    cout << "Headlights Command: " << headlights_command_str << endl;
    if (indicators_command == 0) {
      indicators_command_str = "Disabled";
    }
    else if (indicators_command == 1) {
      indicators_command_str = "Left Turn Indicator";
    }
    else if (indicators_command == 2) {
      indicators_command_str = "Right Turn Indicator";
    }
    else if (indicators_command == 3) {
      indicators_command_str = "Hazard Indicator";
    }
    else {
      indicators_command_str = "Invalid";
    }
    cout << "Indicators Command: " << indicators_command_str << endl;
  }
  json_msg[id + " Throttle"] = to_string(throttle_command); // Throttle command for vehicle
  json_msg[id + " Steering"] = to_string(steering_command); // Steering command for vehicle
  json_msg[id + " Headlights"] = to_string(headlights_command); // Headlights command for vehicle
  json_msg[id + " Indicators"] = to_string(indicators_command); // Indicators command for vehicle
  return json_msg;
}

////////////////////////////////////////////////////////////////////////////////

// TRAFFIC LIGHT CLASS METHODS

// Parse traffic light data
void TrafficLight::parse_data(json data, bool verbose=false) {
  // Traffic light state
  state_str = data[1][id + " State"];
  state = stoi(state_str);
  if (verbose) {
    cout << "\n--------------------------------------" << endl;
    cout << "Receive Data From Traffic Light: " << id << endl;
    cout << "--------------------------------------" << endl;
    // Monitor traffic light data
    if (state == 0) {
      state_str = "Disabled";
    }
    else if (state == 1) {
      state_str = "Red";
    }
    else if (state == 2) {
      state_str = "Yellow";
    }
    else if (state == 3) {
      state_str = "Green";
    }
    else {
      state_str = "Invalid";
    }
    cout << "Traffic Light State: " << state_str << endl;
  }
}

// Generate traffic light control commands
json TrafficLight::generate_commands(json json_msg, bool verbose=false) {
  if (verbose) {
    cout << "\n-------------------------------------" << endl;
    cout << "Transmit Data to Traffic Light: " << id << endl;
    cout << "-------------------------------------" << endl;
    // Monitor traffic light control commands
    if (command == 0) {
      command_str = "Disabled";
    }
    else if (command == 1) {
      command_str = "Red";
    }
    else if (command == 2) {
      command_str = "Yellow";
    }
    else if (command == 3) {
      command_str = "Green";
    }
    else {
      command_str = "Invalid";
    }
    cout << "Traffic Light Command: " << state_str << endl;
  }
  json_msg[id + " State"] = to_string(command); // Command for traffic light
  return json_msg;
}
