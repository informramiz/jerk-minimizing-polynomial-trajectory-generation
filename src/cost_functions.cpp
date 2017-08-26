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

double CostFunctions::s_diff_cost(const Trajectory &trajectory,
                                  int target_vehicle_id,
                                  const VectorXd &delta,
                                  double T,
                                  const vector<Vehicle> &predictions) {

  //as there is separate function for time_diff cost
  //so in this function we will consider time of trajectory
  //as the right time so
  T = trajectory.t;

  VectorXd s_coeffs = trajectory.s_coeffs;
  //calculate value of s-coordinate at time T
  double s_at_t = Utils::solve_polynomial(s_coeffs, T);

  //we also need speed and acceleration values to complete tuple (s, v, a) so
  //make polynomial function for speed which is differentiation of distance (d_coeffs) function
  VectorXd s_dot_coeffs = Utils::differentiate(s_coeffs);
  //calculate speed a time T
  double v_at_t = Utils::solve_polynomial(s_dot_coeffs, T);

  //make polynomial function for acceleration which is differentiation of speed function
  VectorXd s_dot_dot_coeffs = Utils::differentiate(s_dot_coeffs);
  double a_at_t = Utils::solve_polynomial(s_dot_dot_coeffs, T);

  //combine s, v, a to make state of ego vehicle
  VectorXd actual_state(3);
  actual_state << s_at_t, v_at_t, a_at_t;

  //get the target vehicle from predictions using target_vehicle_id
  Vehicle target_vehicle = predictions[target_vehicle_id];
  //predict the state of the target vehicle
  VectorXd target_vehicle_state = target_vehicle.state_at(T);
  //add the required difference in (s, v, a) values that ego vehicle has to keep from target vehicle
  VectorXd expected_state = target_vehicle_state + delta;

  //for each value in ego-vehicle state and target state, penalize if it exceeds allowed variance/difference sigma
  double cost = 0.0;
  for (int i = 0; i < actual_state.rows(); ++i) {
    double diff = abs(actual_state[i] - expected_state[i]);
    cost += Utils::logistic(diff/Constants::SIGMA_S[i]);
  }

  return cost;
}

double CostFunctions::collision_cost(const Trajectory &trajectory,
                                     int target_vehicle_id,
                                     const VectorXd &delta,
                                     double T,
                                     const vector<Vehicle> &predictions) {
  //calculate nearest approach of trajectory to any vehicle in predictions
  double nearest_approach = Utils::nearest_approach_to_any_vehicle(trajectory, predictions);

  //check if this is less than 2 * VEHICLE_RADIUS (diameter of vehicle)
  //then we consider it a collision
  if (nearest_approach < 2 * Constants::VEHICLE_RADIUS) {
    return 1.0;
  }

  return 0.0;
}

double CostFunctions::buffer_cost(const Trajectory &trajectory,
                                  int target_vehicle_id,
                                  const VectorXd &delta,
                                  double T,
                                  const vector<Vehicle> &predictions) {
  //calculate nearest approach of trajectory to any vehicle in predictions
  double nearest_approach = Utils::nearest_approach_to_any_vehicle(trajectory, predictions);

  return Utils::logistic(2*Constants::VEHICLE_RADIUS / nearest_approach);
}

double CostFunctions::exceeds_speed_limit_cost(const Trajectory &trajectory,
                                int target_vehicle_id,
                                const VectorXd &delta,
                                double T,
                                const vector<Vehicle> &predictions) {
  //get polynomial function for longitudinal-speed
  //which is differentiation of s-coordinate function
  VectorXd s_dot_coeffs = Utils::differentiate(trajectory.s_coeffs);

  //we will divide total time in 100 steps and check for each timestep in T duration to find
  //out speed (s_dot) for trajectory at each timestep and for given vehicle at that time step
  double max_v = 0;
  for (int i = 0; i < s_dot_coeffs.rows(); ++i) {
    //consider i% of total time for each iteration
    double t = (T/100) * i;

    //calculate speed at time step t
    double v_at_t = abs(Utils::solve_polynomial(s_dot_coeffs, t));

    //we are only interested in absolute max velocity
    if (v_at_t > max_v) {
      max_v = v_at_t;
    }
  }

  if (max_v > Constants::SPEED_LIMIT) {
    return 1.0;
  }

  return 0.0;
}
