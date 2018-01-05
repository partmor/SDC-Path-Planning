/*
 * behaviour_planner.cpp
 *
 *  Created on: 3 Jan 2018
 *      Author: pedro
 */

#include "behaviour_planner.h"

BehaviourPlanner::BehaviourPlanner(){}

bool BehaviourPlanner::is_lane_safe(LaneKinematics lane_kinematics){
  double safety_gap_ahead = 20;
  double safety_gap_behind = 10;

  bool gap_ahead_ok = lane_kinematics.gap_ahead > safety_gap_ahead;
  bool gap_behind_ok = lane_kinematics.gap_behind > safety_gap_behind;

  bool safe_lane = gap_ahead_ok && gap_behind_ok;
  return safe_lane;
}

FSMState BehaviourPlanner::get_target_state(EgoVehicle &car){
  double dist_pass = 20;
  double delta_v_safe = 0.85;
  double d_tol = 0.5;
  FSMState res;

  if(car.fsm_state.changing_lane){
    double d_obj = 2 + 4 * car.fsm_state.lane_obj;
    double d_diff = fabs(car.prev_path.end_d - d_obj);
    if(d_diff > d_tol){
      return car.fsm_state;
    }
    car.fsm_state.changing_lane = false;
  }

  LaneKinematics left_lane;
  LaneKinematics right_lane;
  LaneKinematics current_lane = car.get_lane_kinematics(car.state.lane);
  if(current_lane.gap_ahead < dist_pass){
    // try to CHANGE LANE
    switch(current_lane.id){
      case 0:
        // try to TURN RIGHT
        right_lane = car.get_lane_kinematics(current_lane.id + 1);
        if(this->is_lane_safe(right_lane)){
          // TURN RIGHT
          res.lane_obj = right_lane.id;
          res.v_obj = right_lane.v;
          res.changing_lane = true;
        } else {
          // STAY, with a safety factor (lower velocity)
          res.lane_obj = current_lane.id;
          res.v_obj = current_lane.v * delta_v_safe;
          res.changing_lane = false;
        }
        break;
      case 1:
        // try to TURN LEFT
        left_lane = car.get_lane_kinematics(current_lane.id - 1);
        if(this->is_lane_safe(left_lane)){
          // TURN LEFT
          res.lane_obj = left_lane.id;
          res.v_obj = left_lane.v;
          res.changing_lane = true;
        } else {
          // try to TURN RIGHT
          right_lane = car.get_lane_kinematics(current_lane.id + 1);
          if(this->is_lane_safe(right_lane)){
            // TURN RIGHT
            res.lane_obj = right_lane.id;
            res.v_obj = right_lane.v;
            res.changing_lane = true;
          } else {
            // STAY, with a safety factor (lower velocity)
            res.lane_obj = current_lane.id;
            res.v_obj = current_lane.v * delta_v_safe;
            res.changing_lane = false;
          }
        }
        break;
      case 2:
        // try to TURN LEFT
        left_lane = car.get_lane_kinematics(current_lane.id - 1);
        if(this->is_lane_safe(left_lane)){
          // TURN LEFT
          res.lane_obj = left_lane.id;
          res.v_obj = left_lane.v; 
          res.changing_lane = true;
        } else {
          // STAY, with a safety factor (lower velocity)
          res.lane_obj = current_lane.id;
          res.v_obj = current_lane.v * delta_v_safe;
          res.changing_lane = false;
        }
        break;
    }
  } else {
    // car ahead is far enough, stay in lane @ max speed
    res.lane_obj = current_lane.id;
    res.v_obj = ROAD_MAX_VEL;
    res.changing_lane = false;
  }
  return res;
}