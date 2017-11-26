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

  Path generate_path(Vehicle &ego_vehicle,
                     int lane,
                     double ref_vel,
                     Path &previous_path,
                     const MapWaypoints map_wps);
};


#endif /* PATH_GENERATOR_H_ */
