/*
 * behaviour_planner.cpp
 *
 *  Created on: 3 Jan 2018
 *      Author: pedro
 */

#include "behaviour_planner.h"

BehaviourPlanner::BehaviourPlanner(){}

bool BehaviourPlanner::is_lane_safe(LaneKinematics lane_kinematics){
  double safety_gap_ahead = 40;
  double safety_gap_behind = 10;

  bool gap_ahead_ok = lane_kinematics.gap_ahead > safety_gap_ahead;
  bool gap_behind_ok = lane_kinematics.gap_behind > safety_gap_behind;

  bool safe_lane = gap_ahead_ok && gap_behind_ok;
  return safe_lane;
}

FSMState BehaviourPlanner::get_target_state(EgoVehicle &car){
  double d_pass = 20;
  double delta_v_safe = 0.8;
  FSMState res;

  LaneKinematics left_lane;
  LaneKinematics right_lane;
  LaneKinematics current_lane = car.get_lane_kinematics(car.state.lane);
  if(current_lane.gap_ahead < d_pass){
    // try to CHANGE LANE
    switch(current_lane.id){
      case 0:
        // try to TURN RIGHT
        right_lane = car.get_lane_kinematics(current_lane.id + 1);
        if(this->is_lane_safe(right_lane)){
          // TURN RIGHT
          res.lane_obj = right_lane.id;
          res.v_obj = right_lane.v;
        } else {
          // STAY, with a safety factor (lower velocity)
          res.lane_obj = current_lane.id;
          res.v_obj = current_lane.v * delta_v_safe;
        }
        break;
      case 1:
        // try to TURN LEFT
        left_lane = car.get_lane_kinematics(current_lane.id - 1);
        if(this->is_lane_safe(left_lane)){
          // TURN LEFT
          res.lane_obj = left_lane.id;
          res.v_obj = left_lane.v;
        } else {
          // try to TURN RIGHT
          right_lane = car.get_lane_kinematics(current_lane.id + 1);
          if(this->is_lane_safe(right_lane)){
            // TURN RIGHT
            res.lane_obj = right_lane.id;
            res.v_obj = right_lane.v;
          } else {
            // STAY, with a safety factor (lower velocity)
            res.lane_obj = current_lane.id;
            res.v_obj = current_lane.v * delta_v_safe;
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
        } else {
          // STAY, with a safety factor (lower velocity)
          res.lane_obj = current_lane.id;
          res.v_obj = current_lane.v * delta_v_safe;
        }
        break;
    }
  } else {
    // car ahead is far enough, stay in lane @ max speed
    res.lane_obj = current_lane.id;
    res.v_obj = ROAD_MAX_VEL;
  }
  return res;
}