/*
 * base.h
 *
 *  Created on: 22 Nov 2017
 *      Author: pedro
 */

#ifndef BASE_H_
#define BASE_H_

#include "json.hpp"

using namespace std;

#define DT_SIM 0.02;
#define LANE_WIDTH 4.0;
#define ROAD_MAX_VEL 21.87; // in m/s,  2.24 mph = 1 m/s

struct MapWaypoints {
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> dx;
  vector<double> dy;
};

struct State {
  double x;
  double y;
  double yaw;
  double s;
  double d;
  double v;
  int lane;
};

struct FSMState{
  string state;
  double s_obj;
  double d_obj;
  int lane_obj;
  double v_obj;
  bool changing_lane = false;
};

struct Path {
  vector<double> pts_x;
  vector<double> pts_y;
  double end_s;
  double end_d;

  int size();
  static Path previous_path_from_json(const nlohmann::json &j);
};

struct LaneKinematics {
  int id;
  double gap_ahead;
  double gap_behind;
  double v;
};


#endif /* BASE_H_ */
