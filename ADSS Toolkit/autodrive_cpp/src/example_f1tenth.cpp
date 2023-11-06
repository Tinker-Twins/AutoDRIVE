// Import libraries
#include <uWS/uWS.h>
#include "autodrive.hpp"

////////////////////////////////////////////////////////////////////////////////

int main() {
  // Initialize vehicle(s)
  F1TENTH f1tenth_1;
  f1tenth_1.id = "V1";

  // Initialize WebSocket hub
  uWS::Hub h;

  h.onMessage([&f1tenth_1](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
            f1tenth_1.parse_data(data, true);

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
            f1tenth_1.throttle_command = 1; // [-1, 1]
            f1tenth_1.steering_command = 1; // [-1, 1]

            /////////////////////////////////////////////////////////////////////////////////////////////////////////

            json json_msg; // JSON message to write to the simulator
            json_msg = f1tenth_1.generate_commands(json_msg, true); // Generate vehicle 1 message

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
