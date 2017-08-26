/*
 * cost_functions.cpp
 *
 *  Created on: Aug 25, 2017
 *      Author: ramiz
 */

#include <math.h>
#include "cost_functions.h"
#include "utils.h"
#include "constants.h"

CostFunctions::CostFunctions() {
  // TODO Auto-generated constructor stub

}

CostFunctions::~CostFunctions() {
  // TODO Auto-generated destructor stub
}

double CostFunctions::time_diff_cost(const Trajectory &trajectory,
                                     int target_vehicle_id,
                                     const VectorXd &delta,
                                     double T,
                                     const vector<Vehicle> &predictions) {

  return Utils::logistic(abs(T - trajectory.t) / T);
}

double CostFunctions::d_diff_cost(const Trajectory &trajectory,
                                  int target_vehicle_id,
                                  const VectorXd &delta,
                                  double T,
                                  const vector<Vehicle> &predictions) {

  //as there is separate function for time_diff cost
  //so in this function we will consider time of trajectory
  //as the right time so
  T = trajectory.t;

  VectorXd d_coeffs = trajectory.d_coeffs;
  //calculate value of d-coordinate at time T
  double d_at_t = Utils::solve_polynomial(d_coeffs, T);

  //we also need speed and acceleration values to complete tuple (d, v, a) so
  //make polynomial function for speed which is differentiation of distance (d_coeffs) function
  VectorXd d_dot_coeffs = Utils::differentiate(d_coeffs);
  //calculate speed a time T
  double v_at_t = Utils::solve_polynomial(d_dot_coeffs, T);

  //make polynomial function for acceleration which is differentiation of speed function
  VectorXd d_dot_dot_coeffs = Utils::differentiate(d_dot_coeffs);
  double a_at_t = Utils::solve_polynomial(d_dot_dot_coeffs, T);

  //combine d, v, a to make state of ego vehicle
  VectorXd actual_state(3);
  actual_state << d_at_t, v_at_t, a_at_t;

  //get the target vehicle from predictions using target_vehicle_id
  Vehicle target_vehicle = predictions[target_vehicle_id];
  //predict the state of the target vehicle
  VectorXd target_vehicle_state = target_vehicle.state_at(T);
  //add the required difference in (d, v, a) values that ego vehicle has to keep from target vehicle
  VectorXd expected_state = target_vehicle_state + delta;

  //for each value in ego-vehicle state and target state, penalize if it exceeds allowed variance sigma
  double cost = 0.0;
  for (int i = 0; i < actual_state.rows(); ++i) {
    double diff = abs(actual_state[i] - expected_state[i]);
    cost += Utils::logistic(diff/Constants::SIGMA_D[i]);
  }

  return cost;
}
