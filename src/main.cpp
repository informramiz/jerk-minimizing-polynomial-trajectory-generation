//============================================================================
// Name        : JerkMinimizingTrajectory.cpp
// Author      : Ramiz Raja
// Version     :
// Copyright   : MIT Licensed
// Description : Hello World in C++, Ansi-style
//============================================================================

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

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * @Input: ([0, 10, 0], [10, 10, 0], 1)
 * @output: [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
 *
 */
vector<double> JMT(vector<double> start, vector<double> end, double T) {

  /*
   Calculate the Jerk Minimizing Trajectory that connects the initial state
   to the final state in time T.

   INPUTS

   start - the vehicles start location given as a length three array
   corresponding to initial values of [s, s_dot, s_double_dot]

   end   - the desired end state for vehicle. Like "start" this is a
   length three array.

   T     - The duration, in seconds, over which this maneuver should occur.

   OUTPUT
   an array of length 6, each value corresponding to a coefficent in the polynomial
   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

   EXAMPLE

   > JMT( [0, 10, 0], [10, 10, 0], 1)
   [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */

  double Si = start[0];
  double Si_dot = start[1];
  double Si_dot_dot = start[2];

  double Sf = end[0];
  double Sf_dot = end[1];
  double Sf_dot_dot = end[2];

  //a0 = Si
  double a0 = Si;
  //a1 = Si_dot
  double a1 = Si_dot;
  //a2 = Si_dot_dot/2
  double a2 = Si_dot_dot / 2;

  //C1 = Si + Si_dot*T + Si_dot_dot/2 * T^2
  double C1 = Si + Si_dot * T + (Si_dot_dot/2) * pow(T, 2);

  //C2 = Si_dot + Si_dot_dot*T
  double C2 = Si_dot + Si_dot_dot * T;

  //C3 = Si_dot_dot
  double C3 = Si_dot_dot;

  MatrixXd B(3, 3);
  B << pow(T, 3), pow(T, 4), pow(T, 5),
      3* pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
      6 * T, 12 * pow(T, 2), 20 * pow(T, 3);

  VectorXd s(3);
  s << Sf-C1,
      Sf_dot-C2,
      Sf_dot_dot-C3;

  VectorXd a = B.inverse() * s;

  vector<double> answer = {a0,a1,a2,a[0],a[1],a[2]};
  VectorXd v(6);
  v << a0, a1, a2, answer[3], answer[4], answer[5];

  cout << "answer: \n" << v << endl;
  return answer;
}

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
    vector<double> jmt = JMT(tc[i].start, tc[i].end, tc[i].T);
    bool correct = close_enough(jmt, answers[i]);
    total_correct &= correct;

  }
  if (!total_correct) {
    cout << "Try again!" << endl;
  } else {
    cout << "Nice work!" << endl;
  }
}

void test_cases() {
  VectorXd state1(6);
  state1 << 0, 10, 0, 0, 0, 0;

  VectorXd state2(6);
  state2 << -1, 10, 0, 1, 0, 0;

  Vehicle vehicle1(state1);
  Vehicle vehicle2(state2);
  vector<Vehicle> vehicles = {vehicle1, vehicle2};

  VectorXd coeffs(6);
  coeffs << 0, 10, 0, 2, 0, 5;

  VectorXd s_coeffs = coeffs.head(3);
  VectorXd d_coeffs = coeffs.tail(3);

  Trajectory trajectory(s_coeffs, d_coeffs, 5);

  //vehicle state update
  VectorXd new_state = vehicle1.state_at(5);
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
  assert(closest_approach == 2.0);

  //closet approach to any vehicle
  double closest_approach_to_any_vehicle = Utils::nearest_approach_to_any_vehicle(trajectory, vehicles);
  cout << "closest approach to any vehicle: " << closest_approach_to_any_vehicle << endl;
  assert(closest_approach_to_any_vehicle == 1);
}

int main() {
  test_cases();
  return 0;
}
