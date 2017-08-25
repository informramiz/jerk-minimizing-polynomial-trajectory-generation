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

double Utils::euclidean(int x1, int y1, int x2, int y2) {
  //  int dist_x = abs(x1 - x2);
  //  int dist_y =  abs(y1 - y2);
  //  int dist = dist_x + dist_y;
  //
  //  return dist;

  int dist_x = (x1 - x2);
  int dist_y =  (y1 - y2);
  int squared_dist = pow(dist_x, 2) + pow(dist_y, 2);

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

double Utils::solve_polynomial(VectorXd coeffs, double x) {
  double total = 0.0;
  for (int i = 0; i < coeffs.rows(); ++i) {
    total += coeffs[i] * pow(x, i);
  }

  return total;
}

VectorXd Utils::differentiate(VectorXd coeffs) {
  vector<double> new_coeffs;
  //ignore the first coefficient as it will always be 0 after
  //differentiation. Multiply power/degree of each term with with its coefficient
  for (int i = 1; i < coeffs.rows(); ++i) {
    double new_coeff = i * coeffs[i];
    new_coeffs.push_back(new_coeff);
  }

  return Utils::vectorToVectorXd(new_coeffs);
}



