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

enum Lane {
  LEFT = 2, MIDDLE = 1, RIGHT = 0
};

struct State {
  double x;
  double y;
  double yaw;
  double s;
  double d;
  double v;
};

struct Path {
  vector<double> pts_x;
  vector<double> pts_y;
  double end_s;
  double end_d;

  int size();
  void set_previous_path_from_json(const nlohmann::json &j);
};


#endif /* BASE_H_ */
