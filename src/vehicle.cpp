/*
 * vehicle.cpp
 *
 *  Created on: 22 Nov 2017
 *      Author: pedro
 */

#include "vehicle.h"

Vehicle::Vehicle(const int i){
  this->id = i;
}

Vehicle::~Vehicle(){}

void Vehicle::set_state_from_json(const nlohmann::json &j){

 this->state.x = j[1]["x"];
 this->state.y = j[1]["y"];
 this->state.yaw = j[1]["yaw"];
 this->state.s = j[1]["s"];
 this->state.d = j[1]["d"];
 this->state.v = j[1]["speed"];

}
