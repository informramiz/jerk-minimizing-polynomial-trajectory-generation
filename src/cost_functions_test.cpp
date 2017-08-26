/*
 * cost_functions_test.cpp
 *
 *  Created on: Aug 26, 2017
 *      Author: ramiz
 */

#include <iostream>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <cassert>
#include <iomanip>
#include "Eigen/Dense"
#include "vehicle.h"
#include "utils.h"
#include "trajectory.h"
#include "cost_functions.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void test_all_cost_functions() {
  const double T = 5;
  VectorXd state1(6);
  state1 << 0, 10, 0, 0, 0, 0;

  VectorXd state2(6);
  state2 << -1, 10, 0, 1, 0, 0;

  /**
   * delta - a length 6 array indicating the offset we are aiming for between us
   *    and the target_vehicle. So if at time 5 the target vehicle will be at
   *   [100, 10, 0, 0, 0, 0] and delta is [-10, 0, 0, 4, 0, 0], then our goal
   *   state for t = 5 will be [90, 10, 0, 4, 0, 0]. This would correspond to a
   *   goal of "follow 10 meters behind and 4 meters to the right of target vehicle"
   */
  VectorXd delta(6);
  delta << -10, 0, 0, -4, 0, 0;

  Vehicle vehicle1(state1);
  Vehicle vehicle2(state2);
  vector<Vehicle> vehicles = {vehicle1, vehicle2};

  int target_vehicle_id = 0;

  VectorXd coeffs(6);
  coeffs << 0, 10, 0, 2, 0, 5;

  VectorXd s_coeffs = coeffs.head(3);
  VectorXd d_coeffs = coeffs.tail(3);

  Trajectory trajectory(s_coeffs, d_coeffs, T);
  CostFunctions cost_functions;

  //time_diff cost function test
  double time_diff_cost = cost_functions.time_diff_cost(trajectory, target_vehicle_id, delta, 9, vehicles);
  cout << "Time diff cost: " << time_diff_cost << endl;
  assert( (time_diff_cost - 0.218) < 0.001);

  double d_diff_cost = cost_functions.d_diff_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "d diff cost: " << d_diff_cost << endl;
  assert( (d_diff_cost - 2.999) < 0.001);

  double s_diff_cost = cost_functions.s_diff_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "s diff cost: " << s_diff_cost << endl;
  assert( (s_diff_cost - 0.462) < 0.001);

  double collision_cost = cost_functions.collision_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "collision cost: " << collision_cost << endl;
  assert(collision_cost == 1);

  double buffer_cost = cost_functions.buffer_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "buffer cost: " << buffer_cost << endl;
  assert((buffer_cost - 0.7859) < 0.0001);

  double exceeds_speed_limit_cost = cost_functions.exceeds_speed_limit_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "exceeds speed limit cost: " << exceeds_speed_limit_cost << endl;
  assert(exceeds_speed_limit_cost == 0);

  double efficiency_cost = cost_functions.efficiency_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "Efficiency cost: " << efficiency_cost << endl;
  assert (efficiency_cost == 0.0);

  double max_accel_cost = cost_functions.max_acceleration_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "Max accel cost: " << max_accel_cost << endl;
  assert (max_accel_cost == 0.0);

  double max_jerk_cost = cost_functions.max_jerk_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "Max jerk cost: " << max_jerk_cost << endl;
  assert (max_jerk_cost == 0.0);

  double total_accel_cost = cost_functions.total_acceleration_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "Total accel cost: " << total_accel_cost << endl;
  assert (total_accel_cost == 0);

  double total_jerk_cost = cost_functions.total_jerk_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "Total jerk cost: " << total_jerk_cost << endl;
  assert (total_jerk_cost == 0);

  double total_cost = cost_functions.calculate_cost(trajectory, target_vehicle_id, delta, T, vehicles, true);
  cout << "Total cost: " << total_cost << endl;
  assert((total_cost - 83.7191) < 0.0001);
}


