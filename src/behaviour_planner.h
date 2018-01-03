/*
 * behaviour_planner.h
 *
 *  Created on: 3 Jan 2018
 *      Author: pedro
 */

#ifndef BEHAVIOUR_PLANNER_H_
#define BEHAVIOUR_PLANNER_H_

#include "base.h"
#include "helper.h"
#include "vehicle.h"

using namespace std;

struct BehaviourPlanner {
    BehaviourPlanner();
    bool is_lane_safe(LaneKinematics lane_kinematics);
    FSMState get_target_state(EgoVehicle &ego_vehicle);
};

#endif /* BEHAVIOUR_PLANNER_H_ */
