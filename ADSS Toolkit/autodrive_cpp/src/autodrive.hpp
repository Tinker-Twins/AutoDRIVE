#ifndef AUTODRIVE_HPP
#define AUTODRIVE_HPP

////////////////////////////////////////////////////////////////////////////////

// Import libraries
#include <iostream>
#include <iterator>
#include <sstream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "json.hpp"

////////////////////////////////////////////////////////////////////////////////

// Namespaces
using namespace std;
using namespace cv;
using nlohmann::json;

////////////////////////////////////////////////////////////////////////////////

// Helper function for numeric data manipulation
inline float* string_to_array(string str) {
  int str_length = str.length(); // Get length of string
  float* arr = new float[str_length]; // Create an array of same size as string length
  // String to array conversion
  istringstream ss( str );
  copy(
    istream_iterator <float> (ss),
    istream_iterator <float> (),
    arr
  );
  return arr; // Return array
}

////////////////////////////////////////////////////////////////////////////////

// Helper function(s) for image manipulation
static const string base64_chars =
"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
"abcdefghijklmnopqrstuvwxyz"
"0123456789+/";

static inline bool is_base64(unsigned char c) {
  return (isalnum(c) || (c == '+') || (c == '/'));
}

inline Mat string_to_image(string const& encoded_string) {
  int in_len = encoded_string.size();
  int i = 0;
  int j = 0;
  int in_ = 0;
  unsigned char char_array_4[4], char_array_3[3];
  string decoded_string;
  while (in_len-- && (encoded_string[in_] != '=') && is_base64(encoded_string[in_])) {
    char_array_4[i++] = encoded_string[in_]; in_++;
    if (i == 4) {
        for (i = 0; i < 4; i++)
          char_array_4[i] = base64_chars.find(char_array_4[i]);
        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
        for (i = 0; (i < 3); i++)
          decoded_string += char_array_3[i];
        i = 0;
    }
  }
  if (i) {
    for (j = i; j < 4; j++)
      char_array_4[j] = 0;
    for (j = 0; j < 4; j++)
      char_array_4[j] = base64_chars.find(char_array_4[j]);
    char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
    char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
    char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
    for (j = 0; (j < i - 1); j++)
      decoded_string += char_array_3[j];
  }
  vector<uchar> img_vector(decoded_string.begin(), decoded_string.end());
  Mat img = imdecode(img_vector, IMREAD_UNCHANGED);
  return img;
}

////////////////////////////////////////////////////////////////////////////////

// Helper fuction for checking if the SocketIO event has JSON data
inline string has_data(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  // If there is data, the JSON object in string format will be returned, else an empty string will be returned
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

////////////////////////////////////////////////////////////////////////////////

// Vehicle class
class Vehicle {
  private:
    // Vehicle data
    string throttle_str, steering_str, encoder_ticks_str, encoder_angles_str, position_str, orientation_quaternion_str, orientation_euler_angles_str, angular_velocity_str, linear_acceleration_str, lidar_scan_rate_str, lidar_range_array_str, lidar_intensity_array_str, front_camera_image_str, rear_camera_image_str;
    // Vehicle commands
    string throttle_command_str, steering_command_str, headlights_command_str, indicators_command_str;
  public:
    // Vehicle data
    string id;
    float throttle, steering, lidar_scan_rate;
    float *encoder_ticks, *encoder_angles, *position, *orientation_quaternion, *orientation_euler_angles, *angular_velocity, *linear_acceleration, *lidar_range_array, *lidar_intensity_array;
    Mat front_camera_image, rear_camera_image;
    // Vehicle commands
    float throttle_command, steering_command;
    int headlights_command, indicators_command;

    // Parse vehicle sensor data
    void parse_data(json data, bool verbose);

    // Generate vehicle control commands
    json generate_commands(json json_msg, bool verbose);
};

////////////////////////////////////////////////////////////////////////////////

// Traffic light class
class TrafficLight {
  private:
    // Traffic light data
    string state_str;
    // Traffic light command
    string command_str;
  public:
    // Traffic light data
    string id;
    int state;
    // Traffic light command
    int command;

    // Parse traffic light data
    void parse_data(json data, bool verbose);

    // Generate traffic light control commands
    json generate_commands(json json_msg, bool verbose);
};

////////////////////////////////////////////////////////////////////////////////

#endif // AUTODRIVE_HPP
