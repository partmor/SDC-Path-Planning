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
#define ROAD_MAX_VEL 22.0; // in m/s,  2.24 mph = 1 m/s

struct MapWaypoints {
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> dx;
  vector<double> dy;
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
