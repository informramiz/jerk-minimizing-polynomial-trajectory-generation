//============================================================================
// Name        : JerkMinimizingTrajectory.cpp
// Author      : Ramiz Raja
// Version     :
// Copyright   : MIT Licensed
// Description : Hello World in C++, Ansi-style
//============================================================================

//#define NDEBUG

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <cassert>
#include "Eigen/Dense"
#include "vehicle.h"
#include "utils.h"
#include "trajectory.h"
#include "cost_functions.h"
#include "polynomial_trajectory_generator.h"
#include "goal.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

bool close_enough(vector<double> poly, vector<double> target_poly, double eps =
                      0.01) {

  if (poly.size() != target_poly.size()) {
    cout << "your solution didn't have the correct number of terms" << endl;
    return false;
  }
  for (int i = 0; i < poly.size(); i++) {
    double diff = poly[i] - target_poly[i];
    if (abs(diff) > eps) {
      cout << "at least one of your terms differed from target by more than "
          << eps << endl;
      return false;
    }

  }
  return true;
}

struct test_case {

  vector<double> start;
  vector<double> end;
  double T;
};

vector<vector<double> > answers = { { 0.0, 10.0, 0.0, 0.0, 0.0, 0.0 }, { 0.0,
    10.0, 0.0, 0.0, -0.625, 0.3125 }, { 5.0, 10.0, 1.0, -3.0, 0.64, -0.0432 } };

void testJMT() {
  //create test cases

  vector<test_case> tc;

  test_case tc1;
  tc1.start = {0,10,0};
  tc1.end = {10,10,0};
  tc1.T = 1;
  tc.push_back(tc1);

  test_case tc2;
  tc2.start = {0,10,0};
  tc2.end = {20,15,20};
  tc2.T = 2;
  tc.push_back(tc2);

  test_case tc3;
  tc3.start = {5,10,2};
  tc3.end = {-30,-20,-4};
  tc3.T = 5;
  tc.push_back(tc3);

  bool total_correct = true;
  for (int i = 0; i < tc.size(); i++) {
    class PolynomialTrajectoryGenerator jmt1;
    VectorXd generated_jmt = jmt1.generate_jerk_minimized_trajectory(Utils::vectorToVectorXd(tc[i].start), Utils::vectorToVectorXd(tc[i].end), tc[i].T);
    bool correct = close_enough(Utils::VectorXdToVector(generated_jmt), answers[i]);
    total_correct &= correct;

  }
  if (!total_correct) {
    cout << "Try again!" << endl;
  } else {
    cout << "Nice work!" << endl;
  }
}

void test_cases() {
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

  VectorXd s_coeffs(6);
  s_coeffs << 0, 3, 0, 1, 0, 4;

  VectorXd d_coeffs(6);
  d_coeffs << 0, 1, 0, 2, 0, 5;

//  VectorXd s_coeffs = coeffs.head(3);
//  VectorXd d_coeffs = coeffs.tail(3);

  Trajectory trajectory(s_coeffs, d_coeffs, T);

  //vehicle state update
  VectorXd new_state = vehicle1.state_at(T);
  cout << "Predicted state of vehicle: " << new_state.transpose() << endl;
  VectorXd vehicle_new_state_answer(6);
  vehicle_new_state_answer << 50, 10, 0, 0, 0, 0;
  assert(vehicle_new_state_answer == new_state);

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
  assert(closest_approach == 0);

  //closet approach to any vehicle
  double closest_approach_to_any_vehicle = Utils::nearest_approach_to_any_vehicle(trajectory, vehicles);
  cout << "closest approach to any vehicle: " << closest_approach_to_any_vehicle << endl;
  assert((closest_approach_to_any_vehicle - 1.41421) < 0.00001);

//  Utils::plot_trajectory(trajectory, vehicle2, true);

  CostFunctions cost_functions;

  //time_diff cost function test
  double time_diff_cost = cost_functions.time_diff_cost(trajectory, target_vehicle_id, delta, 9, vehicles);
  cout << "Time diff cost: " << time_diff_cost << endl;
  assert( (time_diff_cost - 0.218) < 0.001);

  double d_diff_cost = cost_functions.d_diff_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "d diff cost: " << d_diff_cost << endl;
  assert( (d_diff_cost - 3) < 0.001);

  double s_diff_cost = cost_functions.s_diff_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "s diff cost: " << s_diff_cost << endl;
  assert( (s_diff_cost - 3) < 0.001);

  double collision_cost = cost_functions.collision_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "collision cost: " << collision_cost << endl;
  assert(collision_cost == 1);

  double buffer_cost = cost_functions.buffer_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "buffer cost: " << buffer_cost << endl;
  assert((buffer_cost - 1) < 0.0001);

  double exceeds_speed_limit_cost = cost_functions.exceeds_speed_limit_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "exceeds speed limit cost: " << exceeds_speed_limit_cost << endl;
  assert(exceeds_speed_limit_cost == 1);

  double efficiency_cost = cost_functions.efficiency_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "Efficiency cost: " << efficiency_cost << endl;
  assert ((efficiency_cost - (-0.759)) < 0.001);

  double max_accel_cost = cost_functions.max_acceleration_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "Max accel cost: " << max_accel_cost << endl;
  assert (max_accel_cost == 1.0);

  double max_jerk_cost = cost_functions.max_jerk_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "Max jerk cost: " << max_jerk_cost << endl;
  assert (max_jerk_cost == 1.0);

  double total_accel_cost = cost_functions.total_acceleration_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "Total accel cost: " << total_accel_cost << endl;
  assert (total_accel_cost == 1);

  double total_jerk_cost = cost_functions.total_jerk_cost(trajectory, target_vehicle_id, delta, T, vehicles);
  cout << "Total jerk cost: " << total_jerk_cost << endl;
  assert (total_jerk_cost == 1);

  cout << endl << endl;

  double total_cost = cost_functions.calculate_cost(trajectory, target_vehicle_id, delta, T, vehicles, true);
  cout << "Total cost: " << total_cost << endl;
  assert((total_cost - 146.24) < 0.0001);
  
//  PolynomialTrajectoryGenerator ptg;
//  Goal goal(state1.head(3), state1.tail(3), T);
//  Goal perturbed_goal = ptg.perturb_goal(goal);
//  cout << "Perturbed goal: ";
//  perturbed_goal.print();
}

int main() {
//  test_cases();
//  testJMT();


//  vehicle = Vehicle([0,10,0, 0,0,0])
//# predictions = {0: vehicle}
//  # target = 0
//  # #delta = [0, 0, 0, 0, 0 ,0]
//  # delta = [-10, 0, 0, -4, 0, 0]
//  # start_s = [10, 10, 0]
//  # start_d = [4, 0, 0]
//  # T = 5.0
//  # best = PTG(start_s, start_d, target, delta, T, predictions)
//  # show_trajectory(best[0], best[1], best[2], vehicle)

  const double T = 5.0;
  VectorXd state(6);
  state << 5, 10, 0, 3, 0, 0;

//  VectorXd state2(6);
//  state2 << -1, 10, 0, 1, 0, 0;

  /**
   * delta - a length 6 array indicating the offset we are aiming for between us
   *    and the target_vehicle. So if at time 5 the target vehicle will be at
   *   [100, 10, 0, 0, 0, 0] and delta is [-10, 0, 0, 4, 0, 0], then our goal
   *   state for t = 5 will be [90, 10, 0, 4, 0, 0]. This would correspond to a
   *   goal of "follow 10 meters behind and 4 meters to the right of target vehicle"
   */
  VectorXd delta(6);
//  delta << -10, 0, 0, -4, 0, 0;
  delta << -10, 0, 0, 0, 0 ,0;

  VectorXd start_s(3);
  start_s << 10, 10, 0;

  VectorXd start_d(3);
  start_d << 4, 0, 0;

  Vehicle vehicle1(state);
//  Vehicle vehicle2(state2);
  vector<Vehicle> predictions = {vehicle1};

  int target_vehicle_id = 0;

  PolynomialTrajectoryGenerator ptg;
  Trajectory best = ptg.generate_trajectory(start_s, start_d, target_vehicle_id, delta, T, predictions);
  Utils::plot_trajectory(best, vehicle1, true);

  return 0;
}
