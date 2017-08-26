/*
 * utils_test.cpp
 *
 *  Created on: Aug 26, 2017
 *      Author: ramiz
 */

#include <iostream>
#include <cmath>
#include <vector>
#include <cassert>
#include "Eigen/Dense"
#include "vehicle.h"
#include "utils.h"
#include "trajectory.h"
#include "cost_functions.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void test_all_utils_methods() {
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

  //logistic function test
  double logistic_value = Utils::logistic(9);
  cout << "logistic: " << logistic_value << endl;
  assert((logistic_value - 0.999) < 0.001);

  //solve polynomial test
  double solved_poly = Utils::solve_polynomial(coeffs, 2);
  cout << "Solved polynomial: " << solved_poly << endl;
  assert(solved_poly == 196);

  //differentiation test
  VectorXd diff_coeffs = Utils::differentiate(coeffs);
  cout << "Differentiation: " << diff_coeffs.transpose() << endl;
  VectorXd diff_answer(5);
  diff_answer << 10, 0, 6, 0, 25;
  assert(diff_coeffs == diff_answer);

  //closest approach to a vehicle test
  double closest_approach = Utils::nearest_approach_to_vehicle(trajectory, vehicle1);
  cout << "closest approach to a vehicle: " << closest_approach << endl;
  assert(closest_approach == 2.0);

  //closet approach to any vehicle
  double closest_approach_to_any_vehicle = Utils::nearest_approach_to_any_vehicle(trajectory, vehicles);
  cout << "closest approach to any vehicle: " << closest_approach_to_any_vehicle << endl;
  assert((closest_approach_to_any_vehicle - 1.41421) < 0.00001);

  Utils::plot_trajectory(trajectory, vehicle2, true);
}


