#include <iostream>
#include <iterator>
#include <sstream>
#include <algorithm>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <uWS/uWS.h>
#include "json.hpp"

using namespace std;
using namespace cv;
using nlohmann::json;

// Helper function for numeric data manipulation
float* string_to_array(string str) {
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

// Helper function for image manipulation
static const string base64_chars =
"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
"abcdefghijklmnopqrstuvwxyz"
"0123456789+/";

static inline bool is_base64(unsigned char c) {
  return (isalnum(c) || (c == '+') || (c == '/'));
}

Mat string_to_image(string const& encoded_string) {
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

// Variables for vehicle sensor data
string throttle_str, steering_angle_str, encoder_ticks_str, encoder_angles_str, position_str, orientation_quaternion_str, orientation_euler_angles_str, angular_velocity_str, linear_acceleration_str, lidar_scan_rate_str, lidar_range_array_str, lidar_intensity_array_str, front_image_str, rear_image_str;
float *encoder_ticks, *encoder_angles, *position, *orientation_quaternion, *orientation_euler_angles, *angular_velocity, *linear_acceleration, *lidar_range_array, *lidar_intensity_array;
float throttle, steering_angle, lidar_scan_rate;
Mat front_image, rear_image;
// Receive vehicle sensor data [function]
void receive_sensor_data(json j, bool verbose=false) {
  // Actuator feedbacks
  throttle_str = j[1]["Throttle"];
  throttle = stof(throttle_str);
  steering_angle_str = j[1]["Steering"];
  steering_angle = stof(steering_angle_str);
  // Wheel encoders
  encoder_ticks_str = j[1]["Encoder Ticks"];
  encoder_ticks = string_to_array(encoder_ticks_str);
  encoder_angles_str = j[1]["Encoder Angles"];
  encoder_angles = string_to_array(encoder_angles_str);
  // IPS
  position_str = j[1]["Position"];
  position = string_to_array(position_str);
  // IMU
  orientation_quaternion_str = j[1]["Orientation Quaternion"];
  orientation_quaternion = string_to_array(orientation_quaternion_str);
  orientation_euler_angles_str = j[1]["Orientation Euler Angles"];
  orientation_euler_angles = string_to_array(orientation_euler_angles_str);
  angular_velocity_str = j[1]["Angular Velocity"];
  angular_velocity = string_to_array(angular_velocity_str);
  linear_acceleration_str = j[1]["Linear Acceleration"];
  linear_acceleration = string_to_array(linear_acceleration_str);
  // LIDAR
  lidar_scan_rate_str = j[1]["LIDAR Scan Rate"];
  lidar_scan_rate = stof(lidar_scan_rate_str);
  lidar_range_array_str = j[1]["LIDAR Range Array"];
  lidar_range_array = string_to_array(lidar_range_array_str);
  lidar_intensity_array_str = j[1]["LIDAR Intensity Array"];
  lidar_intensity_array = string_to_array(lidar_intensity_array_str);
  // Cameras
  front_image_str = j[1]["Front Camera Image"];
  front_image = string_to_image(front_image_str);
  rear_image_str = j[1]["Rear Camera Image"];
  rear_image = string_to_image(rear_image_str);
  if (verbose) {
    // Monitor sensor data
    cout << "Throttle: " << throttle_str << endl;
    cout << "Steering: " << steering_angle_str << endl;
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
    imshow("Front Camera Preview", front_image);
    imshow("Rear Camera Preview", rear_image);
    waitKey(1);
  }
}

// Variables for vehicle control commands
string throttle_command, steering_command, headlights_command, indicators_command;
// Generate vehicle control commands [function]
void generate_control_commands(float throttle_cmd, float steering_cmd, int headlights_cmd, int indicators_cmd, bool verbose=false) {
  throttle_command = to_string(throttle_cmd);
  steering_command = to_string(steering_cmd);
  headlights_command = to_string(headlights_cmd);
  indicators_command = to_string(indicators_cmd);
  if (verbose) {
    // Monitor control commands
    cout << "Throttle Command: " << throttle_command << endl;
    cout << "Steering Command: " << steering_command << endl;
    cout << "Headlights Command: " << headlights_command << endl;
    cout << "Indicators Command: " << indicators_command << endl;
  }
}

// Check if the SocketIO event has JSON data
// If there is data, the JSON object in string format will be returned, else an empty string will be returned
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  h.onMessage([](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) { // Include the custom class instances within []
      // "42" at the start of the message means there's a WebSocket message event
      // 4 signifies a WebSocket message
      // 2 signifies a WebSocket event

      if (length && length > 2 && data[0] == '4' && data[1] == '2') {
        auto s = hasData(string(data));

        if (s != "") {
          auto j = json::parse(s);
          string event = j[0].get<string>();

          if (event == "Bridge") {
            // j[1] is the data JSON object

            /////////////////////////////////////////////////////////////////////////////////////////////////////////
            // PERCEPTION
            /////////////////////////////////////////////////////////////////////////////////////////////////////////

            receive_sensor_data(j, true);

            /*
              Implement peception stack here.
            */

            /////////////////////////////////////////////////////////////////////////////////////////////////////////
            // PLANNING
            /////////////////////////////////////////////////////////////////////////////////////////////////////////

            /*
              Implement planning stack here.
            */

            /////////////////////////////////////////////////////////////////////////////////////////////////////////
            // CONTROL
            /////////////////////////////////////////////////////////////////////////////////////////////////////////

            /*
              Implement control stack here.
            */

            float throttle_cmd = 1; // [-1, 1]
            float steering_cmd = 1; // [-1, 1]
            int headlights_cmd = 1; // [0 = disabled, 1 = low beam, 2 = high beam]
            int indicators_cmd = 3; // [0 = disabled, 1 = left turn indicator, 2 = right turn indicator, 3 = hazard indicator]

            generate_control_commands(throttle_cmd, steering_cmd, headlights_cmd, indicators_cmd, true);

            /////////////////////////////////////////////////////////////////////////////////////////////////////////

            json json_msg; // JSON message to write to the simulator
            json_msg["Throttle"] = throttle_command; // Throttle command for ego vehicle
            json_msg["Steering"] = steering_command; // Steering command for ego vehicle
            json_msg["Headlights"] = headlights_command; // Headlights command for ego vehicle
            json_msg["Indicators"] = indicators_command; // Indicators command for ego vehicle
            auto msg = "42[\"Bridge\"," + json_msg.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    cout << "Connected!" << endl;
  });

  int port = 4567; // Port number
  if (!h.listen(port)) {
    return -1;
  }

  h.run();
}
