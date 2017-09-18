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

const double Lf = 2.67;

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
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v_mph = j[1]["speed"];
          double v = v_mph * 0.44704; // m/s

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

           //Use advanced state vector to figure out the initial state vector
          // For when actuation will really begin.
          double delta_0 = j[1]["steering_angle"];
          double a_0 = j[1]["throttle"];
          double delay = 0.1; //seconds (specied in milliseconds below
          double ds = v*delay + 0.5*a_0*delay*delay;

          double px_delay = px + ds * cos(psi);
          double py_delay = py + ds * sin(psi);
          // Calculate to second order
          double psi_delay = psi + (ds * tan(-delta_0) / Lf);

          
          
          
          double v0 = v + a_0*delay;

          int n_pts = ptsx.size();
          // Change coordinate system
          vector<double> ptsx_car;
          vector<double> ptsy_car;
          for (int i = 0; i < n_pts; i++){
            double dx = ptsx[i] - px_delay;
            double dy = ptsy[i] - py_delay;
            double x_car = dx * cos(0 - psi_delay) - dy * sin(0 - psi_delay);
            double y_car = dx * sin(0 - psi_delay) + dy * cos(0 - psi_delay);
            ptsx_car.push_back(x_car);
            ptsy_car.push_back(y_car);
          }


          Eigen::VectorXd x_to_fit = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx_car.data(), ptsx_car.size());
          
          Eigen::VectorXd y_to_fit = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy_car.data(), ptsy_car.size());
          
          auto coeffs = polyfit(x_to_fit, y_to_fit, 3);





          // The cross track error is calculated by evaluating at polynomial at x, f(x)
          // and subtracting y.
          double cte = polyeval(coeffs, 0);
          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
          //double epsi = phi0-atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*x0*x0);
          double epsi = -atan(coeffs[1]);
          

          Eigen::VectorXd state(6);
          state << 0, 0, 0, v0, cte, epsi; //x, y, psi, v ...

          auto vars = mpc.Solve(state, coeffs);

          double steer_value = vars[0];
          double throttle_value = vars[1];

          steer_value = -steer_value/deg2rad(25);
          steer_value =  (steer_value < -1.0) ? -1.0 : steer_value;
          steer_value =  steer_value > 1.0 ? 1.0 : steer_value;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value; // negative_value is an issue;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          int n_return = vars.size();
          for (int i = 2; i < n_return; i++){
            if(i%2 == 0){
              mpc_x_vals.push_back(vars[i]);
            } else {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double dx_yellow_line = 2.5;
          int num_points_yellow_line = 25;
          for ( int i = 1; i < num_points_yellow_line; i++){
            next_x_vals.push_back(dx_yellow_line*i);
            next_y_vals.push_back(polyeval(coeffs, dx_yellow_line*i));
          }

          //DEBUG: plot the points themsevles
          //next_x_vals = ptsx_car;
          //next_y_vals = ptsy_car;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

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
