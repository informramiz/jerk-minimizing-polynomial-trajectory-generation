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
#include "Goal.h"
#include "trajectory.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

using namespace std;

class PolynomialTrajectoryGenerator {
public:
  PolynomialTrajectoryGenerator();
  virtual ~PolynomialTrajectoryGenerator();

  VectorXd generate_jerk_minimized_trajectory(VectorXd start, VectorXd end, double T);
  Goal perturb_goal(const Goal &goal);

  vector<Goal> generate_perturbed_goals(const Goal &actual_goal);
  vector<Trajectory> generate_trajectories(const VectorXd &start_s, const VectorXd &start_d, const vector<Goal> &goals);
};

#endif /* POLYNOMIAL_TRAJECTORY_GENERATOR_H_ */
