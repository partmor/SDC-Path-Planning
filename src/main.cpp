#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helper.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // set initial lane
  int lane = 1;

  // initial velocity
  double ref_vel = 49.5 / 2.24; // in m/s

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &ref_vel, &lane]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;


          // IMPORTANT
          // to ensure a smooth transition from cycle to cycle, the new path at a given cycle is
          // generated appending new points to the (few) previous path points that were left over
          // from the last cycle.

          // list of (widely) evenly spaced (x,y) reference waypoints to be interpolated by a spline.
          // more points will be generated from this spline in order to control velocity.
          double prev_ref_x, prev_ref_y;
          double ref_x, ref_y, ref_yaw;
          vector<double> ptsx, ptsy;

          int prev_path_size = previous_path_x.size();
          // if there are "enough" points use the previous path's endpoints as starting reference,
          // to make the new path tangent to the previous one
          if(prev_path_size >= 2){
            ref_x = previous_path_x[prev_path_size - 1];
            ref_y = previous_path_y[prev_path_size - 1];

            prev_ref_x = previous_path_x[prev_path_size - 2];
            prev_ref_y = previous_path_y[prev_path_size - 2];

            ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
          }
          // if previous path is "nearly" empty, use the car's actual state as one of the starting
          // reference points
          else{
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = deg2rad(car_yaw);

            // we want the path to be "smooth", so we define the second starting reference
            // waypoint behind the car such that the path is tangent to the car's trajectory
            // TODO: scale the cos and sin?
            prev_ref_x = ref_x - 1.0 * cos(ref_yaw);
            prev_ref_y = ref_y - 1.0 * sin(ref_yaw);
          }

          ptsx.push_back(prev_ref_x);
          ptsx.push_back(ref_x);

          ptsy.push_back(prev_ref_y);
          ptsy.push_back(ref_y);

          // define rest of reference waypoints for the spline, ahead from the starting reference
          // points just defined, evenly spaced. 5 (2 + 3) points in total are used.
          for(int i = 0; i < 3; i++){
            double wp_s = car_s + (i + 1) * 30;
            double wp_d = 2 + 4 * lane;
            vector<double> ref_wp = getXY(wp_s, wp_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            ptsx.push_back(ref_wp[0]);
            ptsy.push_back(ref_wp[1]);
          }

          // transform to new local coordinates (easier to handle further calculations on the spline):
          // - origin: position of car OR end point of the previous path
          // - orientation: car's yaw OR tangent to the ending of the previous path
          for(int i = 0; i < ptsx.size(); i++){
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // instantiate spline
          tk::spline s;

          // set spline reference points
          s.set_points(ptsx, ptsy);

          // points that will actually be used for the planner
          vector<double> next_x_vals, next_y_vals;

          // start by "recycling the leftovers" of the previous path
          for(int i = 0; i < prev_path_size; i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // break up the spline in a set of points such that the car travels at the desired velocity
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          // fill up the rest of the path planner (up to 50 points)
          for(int i = 0; i <= 50 - prev_path_size; i++){
            double N = target_dist / (.02 * ref_vel);
            double x_ref = (i + 1) * target_x / N;
            double y_ref = s(x_ref);

            // switch back to global coordinates
            double x_point = ref_x + x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            double y_point = ref_y + x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
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
