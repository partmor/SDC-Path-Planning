/*
 * path_generator.cpp
 *
 *  Created on: 23 Nov 2017
 *      Author: pedro
 */

#include "path_generator.h"

PathGenerator::PathGenerator(){}

Path PathGenerator::generate_path(EgoVehicle &ego_vehicle,
                                  Path &previous_path,
                                  const MapWaypoints map_wps){

  int prev_path_size = previous_path.size();

  // IMPORTANT:
  // to ensure a smooth transition from cycle to cycle, the new path at a given cycle is
  // generated appending new points to the (few) previous path points that were left over
  // from the last cycle.

  // list of (x,y) reference waypoints to be interpolated by a spline.
  // more points will be generated from this spline in order to control velocity.
  vector<double> spline_pts_x, spline_pts_y;
  // first two reference waypoints
  double prev_ref_x, prev_ref_y;
  double ref_x, ref_y, ref_yaw;

  // velocity at the end of remaining path portion
  double ref_vel;

  // if there are "enough" points:
  // * use the previous path's endpoints as starting reference (to make the new path
  //   tangent to the previous one)
  // * ref_vel inferred from the last two points knowing that dt = 0.02s
  if(prev_path_size >= 2){
    ref_x = previous_path.pts_x[prev_path_size - 1];
    ref_y = previous_path.pts_y[prev_path_size - 1];

    prev_ref_x = previous_path.pts_x[prev_path_size - 2];
    prev_ref_y = previous_path.pts_y[prev_path_size - 2];

    ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

    ref_vel = sqrt(pow(ref_x - prev_ref_x, 2) + pow(ref_y - prev_ref_y, 2)) / (double)DT_SIM;
  }
  // if previous path is "nearly" empty:
  // * use the car's actual state as one of the starting reference points and place
  //   the other "just behind"
  // * ref_vel equal to actual velocity
  else{
    ref_x = ego_vehicle.state.x;
    ref_y = ego_vehicle.state.y;
    ref_yaw = deg2rad(ego_vehicle.state.yaw);

    // 2nd waypoint behind the car such that the path is tangent to the car's trajectory
    // TODO: scale the cos and sin? try with v * dt.
    prev_ref_x = ref_x - 1.0 * cos(ref_yaw);
    prev_ref_y = ref_y - 1.0 * sin(ref_yaw);

    ref_vel = ego_vehicle.state.v;
  }

  spline_pts_x.push_back(prev_ref_x);
  spline_pts_x.push_back(ref_x);

  spline_pts_y.push_back(prev_ref_y);
  spline_pts_y.push_back(ref_y);

  // if objective speed is not achieved at the end of previous path, accelerate
  if(ego_vehicle.fsm_state.v_obj > ref_vel){
    ref_vel += 0.1;
  } else {
    ref_vel -= 0.1;
  }

  // define rest of reference waypoints for the spline, ahead from the starting reference
  // points just defined, evenly spaced. 5 (2 + 3) points in total are used.
  double start_s = ego_vehicle.state.s;
  if(prev_path_size > 0){
    start_s = previous_path.end_s;
  }
  for(int i = 0; i < 3; i++){
    double wp_s = start_s + (i + 1) * 30;
    double wp_d = 2 + 4 * ego_vehicle.state.lane;
    vector<double> ref_wp = getXY(wp_s, wp_d, map_wps.s, map_wps.x, map_wps.y);
    spline_pts_x.push_back(ref_wp[0]);
    spline_pts_y.push_back(ref_wp[1]);
  }

  // transform to new local coordinates (easier to handle further calculations on the spline):
  // - origin: position of car OR end point of the previous path
  // - orientation: car's yaw OR tangent to the ending of the previous path
  for(int i = 0; i < spline_pts_x.size(); i++){
    double shift_x = spline_pts_x[i] - ref_x;
    double shift_y = spline_pts_y[i] - ref_y;

    spline_pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    spline_pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }

  // instantiate spline
  tk::spline spl;

  // set spline reference points
  spl.set_points(spline_pts_x, spline_pts_y);

  // points that will actually be used for the planner
  Path path;

  // start by "recycling the leftovers" of the previous path
  for(int i = 0; i < prev_path_size; i++){
    path.pts_x.push_back(previous_path.pts_x[i]);
    path.pts_y.push_back(previous_path.pts_y[i]);
  }

  // break up the spline in a set of points such that the car travels at the desired velocity
  double target_x = 30;
  double target_y = spl(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  // fill up the rest of the path planner (up to 50 points)
  for(int i = 0; i <= 50 - prev_path_size; i++){
    double N = target_dist / (.02 * ref_vel);
    double x_ref = (i + 1) * target_x / N;
    double y_ref = spl(x_ref);

    // switch back to global coordinates
    double x_point = ref_x + x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    double y_point = ref_y + x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    path.pts_x.push_back(x_point);
    path.pts_y.push_back(y_point);
  }
  return path;
}
