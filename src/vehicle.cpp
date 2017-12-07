/*
 * vehicle.cpp
 *
 *  Created on: 22 Nov 2017
 *      Author: pedro
 */

#include "vehicle.h"

/*
 * Vehicle
 */
Vehicle::Vehicle(){}
Vehicle::Vehicle(const int id){
  this->id = id;
}

Vehicle::~Vehicle(){}

/*
 * OtherVehicle
 */
OtherVehicle::OtherVehicle() : Vehicle::Vehicle(){}
OtherVehicle::OtherVehicle(const int id) : Vehicle::Vehicle(id){}

void OtherVehicle::predict_state(const double t_horizon){
  State current_state = this->state;
  State pred_state;
  pred_state.s = current_state.s + current_state.v * t_horizon;
  pred_state.d = current_state.d;
  pred_state.v = current_state.v;
}

vector<OtherVehicle> OtherVehicle::from_sensor_fusion_json(const nlohmann::json &j){
  auto sensor_fusion = j[1]["sensor_fusion"];
  vector<OtherVehicle> other_vehicles;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    // instantiate vehicle via id
    int id = sensor_fusion[i][0];
    OtherVehicle other_vehicle = OtherVehicle(id);

    // retrieve position and velocity from sensor measurement
    State state;
    state.x = sensor_fusion[i][1];
    state.y = sensor_fusion[i][2];
    state.s = sensor_fusion[i][5];
    state.d = sensor_fusion[i][6];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    state.v = sqrt(vx*vx + vy*vy);

    // infer lane number using d coordinate
    state.lane = (int) state.d / LANE_WIDTH;

    // set vehicle state
    other_vehicle.state = state;

    other_vehicles.push_back(other_vehicle);
  }

  return other_vehicles;
}

/*
 * EgoVehicle
 */
EgoVehicle::EgoVehicle() : Vehicle::Vehicle(0){}

void EgoVehicle::set_state_from_simulator_json(const nlohmann::json &j){
  this->state.x = j[1]["x"];
  this->state.y = j[1]["y"];
  this->state.yaw = j[1]["yaw"];
  this->state.s = j[1]["s"];
  this->state.d = j[1]["d"];
  this->state.v = j[1]["speed"];

  // infer lane number using d coordinate
  this->state.lane = (int) this->state.d / LANE_WIDTH;
}
