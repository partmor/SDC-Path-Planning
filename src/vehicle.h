/*
 * vehicle.h
 *
 *  Created on: 22 Nov 2017
 *      Author: pedro
 */

#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <string>
#include <limits>
#include "base.h"
#include "helper.h"

using namespace std;

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
};

struct Vehicle{
  int id;
  State state;

  Vehicle();
  Vehicle(const int id);
  virtual ~Vehicle();

};

struct OtherVehicle : Vehicle{
  State predicted_state;

  OtherVehicle();
  OtherVehicle(const int id);
  void predict_state_cv_nlc(const double t_horizon);
  static vector<OtherVehicle> from_sensor_fusion_json(const nlohmann::json &j);
};

struct EgoVehicle : Vehicle{
  FSMState fsm_state;
  vector<OtherVehicle> other_vehicles;

  EgoVehicle();
  void set_state_from_simulator_json(const nlohmann::json &j);
  void detect_other_vehicles_from_sensor_json(const nlohmann::json &j);
  bool get_vehicle_ahead(int search_lane, OtherVehicle &vehicle_ahead);
};



#endif /* VEHICLE_H_ */
