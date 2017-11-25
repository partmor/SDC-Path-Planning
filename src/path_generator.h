/*
 * path_generator.h
 *
 *  Created on: 23 Nov 2017
 *      Author: pedro
 */

#ifndef PATH_GENERATOR_H_
#define PATH_GENERATOR_H_

#include "base.h"
#include "helper.h"
#include "vehicle.h"
#include "spline.h"

struct PathGenerator{
  PathGenerator();

  Path generate_path(Vehicle &egoVehicle,
                     int lane,
                     double ref_vel,
                     Path &previous_path,
                     const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y);
};


#endif /* PATH_GENERATOR_H_ */
