/*
 * vehicle.h
 *
 *  Created on: 22 Nov 2017
 *      Author: pedro
 */

#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <string>
#include "base.h"

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
  void predict_state(const double t_horizon);
  static vector<OtherVehicle> from_sensor_fusion_json(const nlohmann::json &j);
};

struct EgoVehicle : Vehicle{
  FSMState fsm_state;

  EgoVehicle();
  void set_state_from_simulator_json(const nlohmann::json &j);
};



#endif /* VEHICLE_H_ */
