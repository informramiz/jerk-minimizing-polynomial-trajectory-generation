/*
 * jmt.h
 *
 *  Created on: Aug 26, 2017
 *      Author: ramiz
 */

#include <iostream>
#include <vector>
#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;

using namespace std;

#ifndef JMT_H_
#define JMT_H_

class JMT {
public:
  JMT();
  virtual ~JMT();

  VectorXd generate_jmt(VectorXd start, VectorXd end, double T);
};

#endif /* JMT_H_ */
