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
using namespace Eigen;

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

	  // vehicle state (x, y, psi, v)
	  double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v_mph = j[1]["speed"]; // mile per hour
          double v_mps = v_mph * 1600 / 3600; // meter per sec	  
	  double steering_angle = j[1]["steering_angle"];
	  double throttle = j[1]["throttle"];
	  
          /*
          * Calculate reference data from current position (vehicle coordinate)
          */
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
	  int N_waypoint = ptsx.size(); // number of the waypoints
	  VectorXd x_veh(N_waypoint);
	  VectorXd y_veh(N_waypoint);
	  // ref data needs to be transformed from global to vehicle coordinate
	  for(int i=0; i<N_waypoint; i++){
	    double dx = ptsx[i] - px;
	    double dy = ptsy[i] - py;
	    x_veh[i] = dx * cos(-psi) - dy * sin(-psi);
	    y_veh[i] = dx * sin(-psi) + dy * cos(-psi);
	  }	  
          auto coeffs = polyfit(x_veh, y_veh, 3); // Fit waypoints with 3rd polynomial

	  /* 
	   * Calculate Error
	   */
	  //
	  // Now, reference line is given by f(x) = a*x^3 + b*x^2 + c*x + d.
	  // In vehicle coordinate, vehicle is at (x,y)=(0,0) and angle is at +x axis.
	  // When x=0, f(0) is the difference between vehicle and reference : cte.
	  // Similarly, f'(0) is the epsi. Note the sign needs to be inverted.
	  //
	  const double cte = coeffs[0];
          const double epsi = - atan(coeffs[1]);

          /*
          * Calculate steering angle and throttle using MPC. Both are in between [-1, 1].
          */

	  // in vehicle coordinate, psi[t] = 0
	  // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt = 0 + v[t] * dt
	  // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt = 0 + 0
	  // psi_[t+1] = psi[t] - v[t] / Lf * delta[t] * dt = - v / Lf * delta * dt
	  // v_[t+1] = v[t] + a[t] * dt = v + a*dt
	  // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
	  // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
	  
	  Eigen::VectorXd state(6);
	  //state << px, py, psi, v_mps, cte, epsi;

          const double px_act = v_mps * DT;
          const double py_act = 0;
          const double psi_act = - v_mps * steering_angle * DT / LF;
          const double v_act = v_mps + throttle * DT;
          const double cte_act = cte + v_mps * sin(epsi) * DT;
          const double epsi_act = epsi + psi_act; 
          state << px_act, py_act, psi_act, v_act, cte_act, epsi_act;
	  
	  auto mpc_results = mpc.Solve(state, coeffs);

          double steer_value = mpc_results[0]/ deg2rad(25); // convert to [-1..1] range
          double throttle_value = mpc_results[1];
          json msgJson;
          msgJson["steering_angle"] = - steer_value;
          msgJson["throttle"] = throttle_value;
	  
          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals = mpc.mpc_x;
          vector<double> mpc_y_vals = mpc.mpc_y;
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
	  
          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for(int i = 0; i<ptsx.size();i++){
            next_x_vals.push_back(x_veh[i]);
            next_y_vals.push_back(y_veh[i]);
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
