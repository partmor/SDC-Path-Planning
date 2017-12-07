/*
 * helper.h
 *
 *  Created on: Nov 19, 2017
 *      Author: partmor
 */

#ifndef HELPER_H_
#define HELPER_H_

#include <math.h>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for converting back and forth between radians and degrees
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

// for converting back and forth between m/s and mph
double ms2mph(double x);
double mph2ms(double x);

// Euclidean distance between two points, given their (x,y) coordinates
double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> get_JMT(vector<double> start, vector<double> end, double T);

#endif /* HELPER_H_ */
