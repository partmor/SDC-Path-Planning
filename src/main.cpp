#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <limits>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "helper.h"
#include "base.h"
#include "vehicle.h"
#include "path_generator.h"

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
  MapWaypoints map_wps;

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
  	map_wps.x.push_back(x);
  	map_wps.y.push_back(y);
  	map_wps.s.push_back(s);
  	map_wps.dx.push_back(d_x);
  	map_wps.dy.push_back(d_y);
  }

  // ego vehicle
  EgoVehicle car = EgoVehicle();

  h.onMessage([&map_wps, &car]
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
          
        	// get main car's state (position, orientation, velocity)
          car.set_state_from_simulator_json(j);

          // get last cycle's remaining (not executed) fraction of the path
          car.set_previous_path_from_simulator_json(j);

          // get rest of vehicles in the road, detected by sensors
          car.detect_other_vehicles_from_sensor_json(j);

          // get vehicle ahead in current lane
          OtherVehicle vehicle_ahead_cl;
          bool found_vehicle_ahead_cl = car.get_vehicle_ahead_or_behind(car.state.lane, true, vehicle_ahead_cl);

          // get vehicle behind in current lane
          OtherVehicle vehicle_behind_cl;
          bool found_vehicle_behind_cl = car.get_vehicle_ahead_or_behind(car.state.lane, false, vehicle_behind_cl);

          cout << "----------------------------" << endl;
          cout << "prev. path size: " << car.prev_path.size() << endl;

          // keep lane or change to left lane
          double max_vel = mph2ms(48.0);
          if(found_vehicle_ahead_cl){
            bool near = (vehicle_ahead_cl.state.s - car.state.s) < 50;
            if(near){
              car.fsm_state.v_obj = vehicle_ahead_cl.state.v;
            } else {
              car.fsm_state.v_obj = max_vel;
            }
            car.fsm_state.lane_obj = car.state.lane;
            // if vehicle ahead is near, change lane (left)
            if(car.state.lane != 0 && near){
              cout << "CHANGING TO LEFT LANE" << endl;
              int left_lane = car.state.lane - 1;
              car.fsm_state.lane_obj = left_lane;
              // get vehicle ahead in left lane, to set suitable velocity
              OtherVehicle vehicle_ahead_ll;
              bool found_vehicle_ahead_ll = car.get_vehicle_ahead_or_behind(left_lane, true, vehicle_ahead_ll);
              if(found_vehicle_ahead_ll){
                car.fsm_state.v_obj = vehicle_ahead_ll.state.v;
              } else {
                car.fsm_state.v_obj = max_vel;
              }
            }
            double dist_ahead = vehicle_ahead_cl.state.s - car.state.s;
            cout << "distance vehicle ahead: " << dist_ahead << endl;
          } else {
            car.fsm_state.lane_obj = car.state.lane;
            car.fsm_state.v_obj = max_vel;
            cout << "distance vehicle ahead: NA" << endl;
          }
          if(found_vehicle_behind_cl){
            double dist_behind = car.state.s - vehicle_behind_cl.state.s;
            cout << "distance vehicle behind: " << dist_behind << endl;
          }

          cout << "current v: " << car.state.v << endl;
          cout << "desired v: " << car.fsm_state.v_obj << endl;

          // trajectory generator
          PathGenerator path_generator;
          Path new_path = path_generator.generate_path(car, map_wps);

          // push generated path back to simulator
          json msgJson;
          msgJson["next_x"] = new_path.pts_x;
          msgJson["next_y"] = new_path.pts_y;

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
