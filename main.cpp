#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

//Hold previous actuation values - to be used to account for latency
const double Lf = 2.67;
double current_delta = 0;
double current_a = 0;
double latency = 0.1; // 100 ms

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value;
          double throttle_value;

          // Adjusting for 100 ms latency
          // Recall the equations for the model:
      	  // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
          // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
          // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
          // v_[t+1] = v[t] + a[t] * dt
          // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
          // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
          px += v * cos(psi) * latency;
          py += v * sin(psi) * latency;
          //In the simulator however, a positive value implies a right turn and a negative value implies a left turn.
      	  //Therefore, we need to replace the "+" with the "-" sign in the equation below
          psi -= v / Lf * current_delta * latency;
          v += current_a * latency;

          //Converting coordinates from map to car view:
          for (int i = 0; i<ptsx.size(); i++){
        	  // 1. Centering coordinates, i.e. (px, py) is (0,0) in car's coordinate system
        	  double x = ptsx[i] - px;
        	  double y = ptsy[i] - py;
        	  //2. Rotating counter-clockwise (-psi)
        	  ptsx[i] = x * cos(-psi) - y * sin(-psi);
        	  ptsy[i] = x * sin(-psi) + y * cos(-psi);
          }

          //Convert waypoints to Eigen
          //Pointers to first elements
          double* ptr_x = &ptsx[0];
          double* ptr_y = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsx_eigen(ptr_x, 6);
		  Eigen::Map<Eigen::VectorXd> ptsy_eigen(ptr_y, 6);

          // Fit a 3rd degree polynomial to the waypoints
          Eigen::VectorXd coeffs = polyfit(ptsx_eigen, ptsy_eigen, 3);

          // The cross track error is calculated by evaluating at polynomial at x, f(x)
  		  // and subtracting y, and in car's coordinate system x = 0, y = 0;
          double cte = polyeval(coeffs, 0) - 0; //

          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x + coeffs[2]*x² + coeffs[3]*x³:
          //-> coeffs[1] + coeffs[2]*2*x + coeffs[3]*3*x²
          //In car coordinates psi = 0 and x = 0, therefore the formula can be simplified
          double epsi = 0 - atan(coeffs[1]);

          Eigen::VectorXd state(6);
          //x, y, psi = 0 in the car's coordinate system
          state << 0, 0, 0, v, cte, epsi;

          //Store the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //Store actuators & MPC predicted trajectory - which is passed by reference
          auto actuators = mpc.Solve(state, coeffs, mpc_x_vals, mpc_y_vals);

          // Update actuation values to be used to account for latency in the next run
          current_delta = actuators[0]; //radians
          current_a = actuators[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          steer_value = actuators[0] / deg2rad(25); // [-1, 1]
          throttle_value = actuators[1];

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          //unity value of 1 equals 1 meter, so I'm predicting 49 meters...
          for (double i = 0; i < 50; i++) {
    		next_x_vals.push_back(i);
    		next_y_vals.push_back(polyeval(coeffs, i));
  		  }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
