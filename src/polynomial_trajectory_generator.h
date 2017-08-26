/*
 * jmt.h
 *
 *  Created on: Aug 26, 2017
 *      Author: ramiz
 */

#ifndef POLYNOMIAL_TRAJECTORY_GENERATOR_H_
#define POLYNOMIAL_TRAJECTORY_GENERATOR_H_

#include <iostream>
#include <vector>
#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;

using namespace std;

class PolynomialTrajectoryGenerator {
public:
  PolynomialTrajectoryGenerator();
  virtual ~PolynomialTrajectoryGenerator();

  VectorXd generate_jerk_minimized_trajectory(VectorXd start, VectorXd end, double T);
};

#endif /* POLYNOMIAL_TRAJECTORY_GENERATOR_H_ */
