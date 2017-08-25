/*
 * vehicle.h
 *
 * Helper class. Non-ego vehicles move w/ constant acceleration
 *
 *  Created on: Aug 25, 2017
 *      Author: ramiz
 */

#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <iostream>
#include "Eigen/Dense"

using namespace std;
using Eigen::VectorXd;


class Vehicle {
public:
  Vehicle(VectorXd start_state);
  virtual ~Vehicle();

  /**
   * Predict and returns the state of the vehicle at given timestep
   */
  VectorXd state_at(double t) const;

private:
  VectorXd start_state_;

  /**
   * predict new state (d, v, a) at timestep t
   * using equations of motion
   * given old state (d, v, a)
   */
  Eigen::VectorXd predict_coordinate_state(const Eigen::VectorXd& s, double t) const;
};

#endif /* VEHICLE_H_ */
