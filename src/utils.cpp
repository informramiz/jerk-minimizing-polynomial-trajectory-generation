/*
 * utils.cpp
 *
 *  Created on: Aug 21, 2017
 *      Author: ramiz
 */

#include <math.h>
#include "utils.h"

Utils::Utils() {
  // TODO Auto-generated constructor stub

}

Utils::~Utils() {
  // TODO Auto-generated destructor stub
}

double Utils::euclidean(double x1, double y1, double x2, double y2) {
  //  int dist_x = abs(x1 - x2);
  //  int dist_y =  abs(y1 - y2);
  //  int dist = dist_x + dist_y;
  //
  //  return dist;

  double dist_x = (x1 - x2);
  double dist_y =  (y1 - y2);
  double squared_dist = pow(dist_x, 2) + pow(dist_y, 2);

  return sqrt(squared_dist);
}

double Utils::euclidean_3d(int x1, int y1, double theta_rad1, int x2, int y2, double theta_rad2) {
  int dist_x = (x1 - x2);
  int dist_y =  (y1 - y2);
  double dist_theta = (theta_rad1 - theta_rad2);
  double squared_dist = pow(dist_x, 2) + pow(dist_y, 2) + pow(dist_theta, 2);

  return sqrt(squared_dist);
}

double Utils::deg2rad(double delta_i) {
  return M_PI / 180.0 * delta_i;
}

double Utils::logistic(double x) {
  return (2.0 / (1 + exp(-x))) - 1.0;
}

double Utils::solve_polynomial(const VectorXd &coeffs, double x) {
  double total = 0.0;
  for (int i = 0; i < coeffs.rows(); ++i) {
    total += coeffs[i] * pow(x, i);
  }

  return total;
}

VectorXd Utils::differentiate(const VectorXd &coeffs) {
  vector<double> new_coeffs;
  //ignore the first coefficient as it will always be 0 after
  //differentiation. Multiply power/degree of each term with with its coefficient
  for (int i = 1; i < coeffs.rows(); ++i) {
    double new_coeff = i * coeffs[i];
    new_coeffs.push_back(new_coeff);
  }

  return Utils::vectorToVectorXd(new_coeffs);
}

double Utils::nearest_approach_to_vehicle(const Trajectory &trajectory, const Vehicle &vehicle) {

  //assign a very high value initially
  double closest = 999999;

  //we will divide total time in 100 steps and check for each timestep in trajectory.t duration to find
  //out value of (s, d) for trajectory and for given vehicle at that time step
  for (int i = 0; i < 100; ++i) {
    //consider i% of total timestep for eachc iteration
    double t = trajectory.t/100 * i;

    //get s-coordinate value at time t
    //by solving trajectory.s_coeffs polynomial function
    //at time t
    double trajectory_s = Utils::solve_polynomial(trajectory.s_coeffs, t);

    //get d-coordinate value at time t
    //by solving trajectory.s_coeffs polynomial function
    //at time t
    double trajectory_d = Utils::solve_polynomial(trajectory.d_coeffs, t);

    //now predict the state of target vehicle at time t
    VectorXd target_vehicle_state = vehicle.state_at(t);
    //extract (s, d)
    double target_vehicle_s = target_vehicle_state[0];
    double target_vehicle_d = target_vehicle_state[3];

    //calculate the euclidean distance between trajectory's (s, d)
    //and target vehicle's (s, d) at time t
    double distance = euclidean(trajectory_s, trajectory_d, target_vehicle_s, target_vehicle_d);

    if (distance < closest) {
      closest = distance;
    }
  }

  return closest;
}

double Utils::nearest_approach_to_any_vehicle(const Trajectory &trajectory, const vector<Vehicle> &vehicles) {
  double closest_to_any_vehicle = 999999;

  for (int i = 0; i < vehicles.size(); ++i) {
    //calculate closest approach of trajectory to current vehicle
    double closest_to_current_vehicle = nearest_approach_to_vehicle(trajectory, vehicles[i]);

    if (closest_to_current_vehicle < closest_to_any_vehicle) {
      closest_to_any_vehicle = closest_to_current_vehicle;
    }
  }

  return closest_to_any_vehicle;
}




