#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using namespace std;

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
  /**
   * TODO: Initialize the pid variable.
   */
      
  // Show arguments passed
  std::cout << "Please pass 'twiddle' as the first argument to run twiddle parameter search." << std::endl;
  std::cout << "The number of arguments passed: " << argc << std::endl;
  std::cout << "---- Arguments passed (FYI) ----" << std::endl;
  for(int counter=0; counter < argc; counter++)
    std::cout << "argv[" << counter << "]: " << argv[counter] << std::endl;
    
  // Initialize
  vector<double> p = {0.1, 0.001, 2};
  vector<double> dp = {0.01, 0.0001, 0.2};
  
  bool is_twiddle = false;
  if (argc>=2)
    if (std::string(argv[1]) == "twiddle")
      is_twiddle = true;

  int data_cnt = 0;
  int twiddle_cnt = 0;
  int p_index = 0;
  
  bool first_chk = true;
  bool second_chk = true;
  bool third_chk = true;
  
  double error = 0.0;
  double best_err = 0.0;
  int n = 1000;
  double tolerance = 0.1;
  
  if (is_twiddle){
    pid.Init(p[0], p[1] ,p[2]);
  } else {
    // Specify the best hyperparameters found by twiddle here!
    pid.Init(0.109, 0.001, 2.6378);
  }
  
  h.onMessage(
    [&pid, &is_twiddle, &p, &dp, &p_index, &data_cnt, &twiddle_cnt, &error, &n, &best_err, &tolerance, &first_chk, &second_chk, &third_chk](
      uWS::WebSocket <uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          if (is_twiddle && data_cnt==0)
            std::cout << std::endl << "Run with p: " << p[0] << ", " << p[1] << ", " << p[2]<< ", dp: " << dp[0] << ", " << dp[1] << ", " << dp[2] << std::endl;
          
          // Calculate total error
          error += cte * cte;
          
          if (is_twiddle && data_cnt == n){
            std::cout << "## Twiddle cycle: " << twiddle_cnt << ", p index: " << p_index << " started with first_chk[" << first_chk << "], second_chk[" << second_chk << "] and third_chk[" << third_chk << "]" << std::endl;
            error = error / n;
            std::cout << "error: " << error << std::endl;
            if (twiddle_cnt==0 && p_index==0 && first_chk==true)
              best_err = error;
            
            if (first_chk) {
              first_chk = false;
              p[p_index] += dp[p_index];
              pid.Init(p[0], p[1] ,p[2]);
              std::cout << "p updated to: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
            } else {
              if (error < best_err) {
                std::cout << "After increasing p, error [" << error << "] is less than best_err [" << best_err << std::endl;
                best_err = error;
                dp[p_index] *= 1.1;
                std::cout << "dp updated to: " << dp[0] << ", " << dp[1] << ", " << dp[2] << std::endl;
                second_chk = false;
                third_chk = false;
              } else {
                if (second_chk) {
                  second_chk = false;
                  p[p_index] -= 2 * dp[p_index];
                  pid.Init(p[0], p[1] ,p[2]);
                  std::cout << "p updated to: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
                } else {
                  if (error < best_err) {
                    std::cout << "After decreasing p, error [" << error << "] is less than best_err [" << best_err << std::endl;
                    best_err = error;
                    dp[p_index] *= 1.1;
                    std::cout << " and dp increased by 10% to: " << dp[0] << ", " << dp[1] << ", " << dp[2] << std::endl;
                  } else {
                    std::cout << "Did not improve, so set p back to the original value" << std::endl;
                    p[p_index] += dp[p_index];
                    dp[p_index] *= 0.9;
                    std::cout << " and dp decreased by 10% to: " << dp[0] << ", " << dp[1] << ", " << dp[2] << std::endl;
                  }
                  third_chk = false;
                }
              }
            }
            if (!first_chk && !second_chk && !third_chk){
              std::cout << "updating index..." << std::endl;
              p_index += 1;
              first_chk = true;
              second_chk = true;
              third_chk = true;
              if (p_index == 3) {
                double sum_dp = dp[0] + dp[1] + dp[2];
                std::cout << "## Twiddle cycle: " << twiddle_cnt << " completed, sum_dp: "<< sum_dp << std::endl;
                if (sum_dp < tolerance) {
                  std::cout << "### Twiddle Completed ###" << std::endl;
                  std::cout << "Best p: " << p[0] << ", " << p[1] << ", " << p[2]<< ", and dp: " << dp[0] << ", " << dp[1] << ", " << dp[2] << std::endl;
                  ws.close();
                } else {
                  twiddle_cnt += 1;
                  p_index = 0;
                }
              }
            }
            data_cnt = -1;
            error = 0.0;
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          } else {
            json msgJson;
            pid.UpdateError(cte);
            steer_value = pid.TotalError();
            if (steer_value > 1) {
              steer_value = 1;
            } else if (steer_value < -1) {
              steer_value = -1;
            }
            data_cnt += 1;
            
            // DEBUG
            if (!is_twiddle)
              std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Total Error: " << error << std::endl;

            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.6;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
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
