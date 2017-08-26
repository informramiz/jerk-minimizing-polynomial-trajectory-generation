/*
 * goal.h
 *
 *  Created on: Aug 26, 2017
 *      Author: ramiz
 */

#ifndef GOAL_H_
#define GOAL_H_

#include "Eigen/Dense"
using Eigen::VectorXd;

struct Goal {
  VectorXd s;
  VectorXd d;
  double t;

  Goal(const VectorXd &s, const VectorXd &d, double t) {
    this->s = s;
    this->d = d;
    this->t = t;
  }
};


#endif /* GOAL_H_ */
