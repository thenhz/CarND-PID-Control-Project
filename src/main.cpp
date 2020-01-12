#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"

// for convenience
using nlohmann::json;
using std::string;
using namespace std;

// Starting values derived from multiple twiddle runs


static const double SPEED_MAX = 100.0;
static const double SPEED_MIN = 0.0;
static const double THROTTLE_MAX = 1.0;
static const double THROTTLE_MIN = 0.45;
static const double HIGHEST_CTE = 1.0;

static const int SAMPLE_SIZE = 100;
static const double MIN_TOLERANCE = 0.2;

static bool achieved_tolerance_ = false;

static int count_ = 0;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char *argv[]) {
  uWS::Hub h;

  PID pid;
  Twiddle twiddle;

  double init_Kp;
  double init_Ki;
  double init_Kd;
  
  if (argc == 1) {
  	cout << "one parameter" << endl;

  	init_Kp = 0.15;
  	init_Ki = 0.0004;
  	init_Kd = 3;
  }
  else if (argc == 4) {
  	init_Kp = atof(argv[1]);
  	init_Ki = atof(argv[2]);
  	init_Kd = atof(argv[3]);
  }
  else {
  	cerr << "ERROR in input parameters. USAGE1: pid USAGE2: pid Kp Ki Kd" << endl;
    return -1;
  }
  cout << "Load values --- Kp= " << init_Kp << " , Ki= " << init_Ki << " , Kd= " << init_Kd
  		<< endl;

  pid.Init(init_Kp, init_Ki, init_Kd);
  
  twiddle.init(init_Kp, init_Ki, init_Kd);

  h.onMessage([&pid, &twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;

          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          //clip values 
          steer_value = std::max(-1.0, std::min(steer_value, 1.0));

          // let's try to set throttle
          double throttle;
          throttle = THROTTLE_MAX * (HIGHEST_CTE - std::min(HIGHEST_CTE,fabs(cte))) ;          
          // Normalize the throttle 
          throttle = ((THROTTLE_MAX - THROTTLE_MIN) * (throttle - SPEED_MIN)) / (SPEED_MAX - SPEED_MIN) + THROTTLE_MIN;

          /*
          // Twiddle the parameters until tolerance is met
          if (!achieved_tolerance_) {
            twiddle.incrementCount(cte);
            if (++count_ % SAMPLE_SIZE == 0) {
              std::vector<double> params = twiddle.updateParams();
              if (twiddle.getTolerance() < MIN_TOLERANCE) {
                achieved_tolerance_ = true;
              } else {
                pid.Init(params[0], params[1], params[2]);
              }
            }
          }
          */
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}