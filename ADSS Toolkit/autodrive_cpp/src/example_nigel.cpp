// Import libraries
#include <uWS/uWS.h>
#include "autodrive.hpp"

////////////////////////////////////////////////////////////////////////////////

int main() {
  // Initialize vehicle(s)
  Nigel nigel_1;
  nigel_1.id = "V1";

  // Initialize traffic light(s)
  TrafficLight tl_1;
  TrafficLight tl_2;
  TrafficLight tl_3;
  TrafficLight tl_4;
  tl_1.id = "TL1";
  tl_2.id = "TL2";
  tl_3.id = "TL3";
  tl_4.id = "TL4";

  // Initialize WebSocket hub
  uWS::Hub h;

  h.onMessage([&nigel_1, &tl_1, &tl_2, &tl_3, &tl_4](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
      // "42" at the start of the message means there's a WebSocket message event
      // 4 signifies a WebSocket message
      // 2 signifies a WebSocket event
      if (length && length > 2 && data[0] == '4' && data[1] == '2') {
        auto s = has_data(string(data));
        if (s != "") {
          auto data = json::parse(s);
          string event = data[0].get<string>();
          if (event == "Bridge") {
            // data[1] is the data JSON object

            /////////////////////////////////////////////////////////////////////////////////////////////////////////
            // PERCEPTION
            /////////////////////////////////////////////////////////////////////////////////////////////////////////

            // Vehicle data
            nigel_1.parse_data(data, true);

            // Traffic light data
            tl_1.parse_data(data, true);
            tl_2.parse_data(data, true);
            tl_3.parse_data(data, true);
            tl_4.parse_data(data, true);

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

            // Vehicle control
            nigel_1.throttle_command = 1; // [-1, 1]
            nigel_1.steering_command = 1; // [-1, 1]
            nigel_1.headlights_command = 1; // [0 = disabled, 1 = low beam, 2 = high beam]
            nigel_1.indicators_command = 3; // [0 = disabled, 1 = left turn indicator, 2 = right turn indicator, 3 = hazard indicator]

            // Traffic light control
            tl_1.command = 1; // [0 = disabled, 1 = red, 2 = yellow, 3 = green]
            tl_2.command = 2; // [0 = disabled, 1 = red, 2 = yellow, 3 = green]
            tl_3.command = 3; // [0 = disabled, 1 = red, 2 = yellow, 3 = green]
            tl_4.command = 3; // [0 = disabled, 1 = red, 2 = yellow, 3 = green]

            /////////////////////////////////////////////////////////////////////////////////////////////////////////

            json json_msg; // JSON message to write to the simulator
            json_msg = nigel_1.generate_commands(json_msg, true); // Generate vehicle 1 message
            json_msg = tl_1.generate_commands(json_msg, true); // Append traffic light 1 message
            json_msg = tl_2.generate_commands(json_msg, true); // Append traffic light 2 message
            json_msg = tl_3.generate_commands(json_msg, true); // Append traffic light 3 message
            json_msg = tl_4.generate_commands(json_msg, true); // Append traffic light 4 message

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
  cout << "wsgi starting up on http://0.0.0.0:" << port << endl;
  if (!h.listen(port)) {
    return -1;
  }

  h.run();
}
