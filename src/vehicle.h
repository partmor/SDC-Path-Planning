/*
 * vehicle.h
 *
 *  Created on: 22 Nov 2017
 *      Author: pedro
 */

#ifndef VEHICLE_H_
#define VEHICLE_H_

#include "base.h"

using namespace std;

struct Vehicle{

  int id;
  State state;

  Vehicle(const int i);
  virtual ~Vehicle();

  void set_state_from_json(const nlohmann::json &j);

};



#endif /* VEHICLE_H_ */
