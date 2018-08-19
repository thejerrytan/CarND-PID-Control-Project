#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  const double Kp = 1.0/5;
  const double Ki = 0.005;
  const double Kd = 5.0;
  const double dp = 0.02;
  const double di = 0.0005;
  const double dd = 0.05;
  const double tol = 0.01;
  // TODO: Initialize the pid variable.
  pid.Init(Kp, Ki, Kd, dp, di, dd);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value, avg_error, throttle;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          const double sumD = pid.dp + pid.dd + pid.di;
          switch (pid.count % (3 * pid.WINDOW_SIZE)) {
            case (0):
              if (sumD > 0.05) {
                pid.Kp = fabs(pid.Kp + pid.dp);
                steer_value = pid.Run(cte);
                avg_error = pid.AverageError();
                // Here we evaluate the decision taken 100 time steps ago - for Kd
                // The evaluation for Kp would be done in the next case statement
                if (avg_error < pid.best_average) {
                  pid.best_average = avg_error;
                  pid.dd *= 1.1;
                } else {
                  pid.Kd = fabs(pid.Kd - 2 * pid.dd);
                  pid.dd *= 0.9;
                }
              } else {
                steer_value = pid.Run(cte);
                avg_error = pid.AverageError();
              }
              break;
            case (50):
              if (sumD > 0.05) {
                pid.Ki = fabs(pid.Ki + pid.di);
                steer_value = pid.Run(cte);
                avg_error = pid.AverageError();
                if (avg_error < pid.best_average) {
                  pid.best_average = avg_error;
                  pid.dp *= 1.1;
                } else {
                  pid.Kp = fabs(pid.Kp - 2 * pid.dp);
                  pid.dp *= 0.9;
                }
              } else {
                steer_value = pid.Run(cte);
                avg_error = pid.AverageError();
              }
              break;
            case (150):
              if (sumD > 0.05) {
                pid.Kd = fabs(pid.Kd + pid.dd);
                steer_value = pid.Run(cte);
                avg_error = pid.AverageError();
                if (avg_error < pid.best_average) {
                  pid.best_average = avg_error;
                  pid.di *= 1.1;
                } else {
                  pid.Ki = fabs(pid.Ki - 2 * pid.di);
                  pid.di *= 0.9;
                }
              } else {
                steer_value = pid.Run(cte);
                avg_error = pid.AverageError();
              };
            default:
              steer_value = pid.Run(cte);
              avg_error = pid.AverageError();
          }

          // throttle = bias + steer_factor + p - d + i
          // bias -> constant term
          // steer_factor -> A * (1 - |steer_value|), if driving straight should go fast, if turning, should slow down
          // p -> B * |cte| - speed should be proportional to cte, assuming steer_value is correct, the further away
          //      you are from the trajectory, the faster you should correct yourself
          // d -> C * (delta cte), differential term. If cte increases (at a turn for e.g.) should slow down,
          //      if cte decreases (straight road) should speed up
          // i -> D * (longTermAverageError - fmin(avg_error, E)), integral term. If current cte average over last 50 time steps is low,
          //      I should speed up. If current cte average is high, slow down.
          //      fmin(avg_error, E) is to put a ceiling value of E, to prevent cases of extreme high error at the beginning from setting throttle to 0.
          // Hyperparameters A, B, C, D, E are tweaked by hand, longTermAverageError from observations.
          throttle = 0.18 + 0.5 * (1 - fabs(steer_value)) + 1.0/5.0 * fabs(cte) - 5.0 * (cte - pid.prev_cte) + (2.5 * (0.2 - fmin(avg_error,0.4)));
          throttle = fmax(fmin(throttle, 1.0), 0.0);

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          std::cout << "Error: " << pid.TotalError() << ", " << avg_error << ", " << pid.best_average << std::endl;
          std::cout << "Kp, Ki, Kd : " << pid.Kp << ", " << pid.Ki << ", " << pid.Kd << std::endl;
          std::cout << "dp, di, dd : " << pid.dp << ", " << pid.di << ", " << pid.dd << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
