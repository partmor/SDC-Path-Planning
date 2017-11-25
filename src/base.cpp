/*
 * base.cpp
 *
 *  Created on: 23 Nov 2017
 *      Author: pedro
 */

#include "base.h"

int Path::size(){
  return this->pts_x.size();
}

Path Path::previous_path_from_json(const nlohmann::json &j){

  auto prev_path_x = j[1]["previous_path_x"];
  auto prev_path_y = j[1]["previous_path_y"];

  Path path;
  for(int i = 0; i < prev_path_x.size(); i++){
    path.pts_x.push_back(prev_path_x[i]);
    path.pts_y.push_back(prev_path_y[i]);
  }

  path.end_s = j[1]["end_path_s"];
  path.end_d = j[1]["end_path_d"];

  return path;
}

