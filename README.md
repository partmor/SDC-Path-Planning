# **Path Planning**  [![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
[//]: # (Image References)
[placeholder]: ./img/placeholder.png

The goal of this project is to implement a **path planner** prototype for **autonomous highway driving** in the scenario provided by [Udacity's simulator](https://github.com/udacity/self-driving-car-sim/releases).

The program for the path planner, written in C++, communicates with the simulator in order to generate smooth, comfortable and safe paths for the autonomous vehicle, based on localization and sensor fusion data.

The autonomous vehicle, with the help of the path planning system, safely navigates around the virtual highway (50 mph limited) with other traffic that is driving at different *arbitrary* velocities below the track limit. The car tries to go as close as possible to the 50 MPH speed limit, passing slower traffic when possible, without experiencing total acceleration over 10 m/s^2 or jerk greater than 10 m/s^3. The car also avoids hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless it performs a lane change maneuver.

## Inputs

A static [map of the track](data/higway_map.csv) is provided. It contains the geometrical data of a sequence of waypoints located along the center line of the highway:

* `x`, `y`: global coordinates of the waypoint, in meters.
* `s`: Frenet longitudinal displacement coordinate in meters. Ranges from 0 to 6945.554 meters (closed loop).
* `dx`, `dy`: unit normal vector components to the highway's center line, pointing outward of the highway loop.

Additionally, at the beginning of each planning cycle, the simulator serves the program the following inputs in JSON format:

#### Ego vehicle's noiseless localization data (noiseless)

* `x`, `y`: Cartesian global (map) coordinates of the car's latest position, in meters.
* `s`, `d`: Frenet coordinates of the car's latest position, in meters.
* `yaw`: Car's heading angle with respect to global *x* axis, positive counter-clockwise, in degrees.
* `speed`: The car's scalar velocity, in mph.

#### Remaining points from previous path

The vehicle may not have time to execute a complete path or trajectory between two planning cycles. The previous path data consists of the trajectory points generated during the last planning cycle, and that were not executed.

*  `previous_path_x`, `previous_path_y`: lists of *x* and *y* global coordinates of the remaining path points from the previous cycle.

#### Previous path's *s* and *d* end values

* `end_path_s`, `end_path_d`: Frenet coordinates of the endpoint of the path planned in the previous cycle.

#### Noiseless sensor fusion data

* `sensor_fusion`: A 2D vector of all sensed cars in the same side of the track, and their attributes: unique ID, *x* position in map coordinates [meters], *y* position in map coordinates [meters], *x* velocity [m/s], *y* velocity [m/s], *s* position in Frenet coordinates [meters], *d* position in Frenet coordinates [meters]. 

### Additional details about the simulator

* The ego vehicle uses a perfect controller and will visit every `(x, y)` point it receives in the path every 0.02 seconds. Hence, the spacing of the points determines the speed of the car. The vector going from a point to the next one in the planned trajectory dictates the angle of the car. Acceleration both in the tangential and normal directions are measured, along with the jerk (rate of change of total acceleration).

* There is some latency between the start of a planning cycle by the simulator and the path planner returning a path. With *fairly* optimized code this should take around one to three time steps (~ 60 ms). During this delay the simulator will continue visiting points from the latest path it was provided. This is important to consider in order to generate smooth paths.

### 

## Path planning *workflow*

It is easier to visualize the componets that integrate the planning system, and how they connect to eachother, following the program flow as stated in [`main.cpp`](src/main.cpp).

### 1. Data recovery

The data provided by the simulator for the current cycle in JSON format is parsed to extract the current state of the ego vehicle, the remaining path points from previous cycle, and the state of surrounding vehicles detected by sensor fusion.

Two different vehicle classes were defined, both inheriting from a `Vehicle` base class (see definitions in [`vehicle.h`](src/vehicle.h)):

* `EgoVehicle`: allows to keep track of the state of the ego vehicle and encapsulate all functionality oriented to *perceive* the world around it.
* `OtherVehicle`: allows to keep track of the state of all surrounding vehicles detected by sensor fusion.

A single instance of `EgoVehicle`, named `car`, is used throughout the program execution. It is instantiated outside the planning loop and is updated in each cycle. The JSON data parsing is performed by the `EgoVehicle::set_state_from_simulator_json()`, `EgoVehicle::set_previous_path_from_simulator_json()`, and `EgoVehicle::detect_other_vehicles_from_sensor_json()` methods.

### 2. Behaviour planning

The behaviour planner (implemented in [behaviour_planner.cpp](src/behaviour_planner.cpp)) is responsible for providing the target state for the ego vehicle. The `BehaviourPlanner` class implements a primitive Finite State Machine, with three states: 

* Stay in lane
* Change to left lane
* Change to right lane

The simple decision tree that establishes the transition logic between states is implemented in the `BehaviourPlanner::get_target_state()` method. The output of the latter is the target lane and target velocity, encapsulated in a `FSMState` object.

The `car` can measure the state of the current and adjacent lanes via its `EgoVehicle::get_lane_kinematics()` which provides, for a given lane, the available gaps ahead and behind, as well as traffic flow velocity, stored in a `LaneKinematics` object. This lane state is evaluated in the future horizon, i.e, distances and traffic flow velocity are calculated based on the predicted states of the surrounding detected `OtherVehicles` in the future instant corresponding to the end of the previous path. To predict the state of the surrounding traffic, a very simple and naive model was taken for this first version of the program: vehicles move at a constant (scalar) velocity, and will not perform lane changes. This model is implemented in the `OtherVehicle::predict_state_cv_nlc()` method (*cv_nlc* stands for constant velocity no lane change).

Hence, `BehaviourPlanner::get_target_state()` will check the distance with the car in front. If greater than `dist_pass`, the ego vehicle will continue in the current lane at maximum velocity, otherwise it will explore the availability of adjacent lanes, via the `BehaviourPlanner::is_lane_safe()` method, giving preference to left lane changes (left-overtaking is preferred). If the lane change cannot be performed immediately the `car`'s velocity is adjusted to `delta_v_safe` (< 1) times the leading vehicle's velocity in order to avoid collision.

When it is safe to change lane, the target lane is set equal to the lane the car is willing to switch to, and the target velocity is set equal to the traffic flow velocity of the target lane.

### 3. Path generator

Finally, the target state determined by the behaviour planner is fed into the path generator, implemented in [path_generator.cpp](src/path_generator.cpp).

The `PathGenerator::generate_path()` method uses the behaviour planner's target state (stored in `car.fsm_state`) together with the `car`'s current state, the remaining previous path points, and static map waypoints to generate the updated `x`, `y` coordinates of the trajectory the vehicle most follow.

The trajectory is generated *recycling* the remaining points of the previous path in order to achieve a smooth transitions. New points up to the desired future horizon (1 second - 50 points to be visted every 0.02 seconds) are generated applying splines between previous path's endpoint and the target position. A third party [cubic spline library](http://kluge.in-chemnitz.de/opensource/spline/) was used for this task.

Further details about the waypoint interpolation process can be found in the comments in [path_generator.cpp](src/path_generator.cpp).

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`
