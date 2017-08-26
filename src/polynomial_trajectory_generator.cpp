/*
 * jmt.cpp
 *
 *  Created on: Aug 26, 2017
 *      Author: ramiz
 */

#include <math.h>

#include "polynomial_trajectory_generator.h"

PolynomialTrajectoryGenerator::PolynomialTrajectoryGenerator() {
  // TODO Auto-generated constructor stub

}

PolynomialTrajectoryGenerator::~PolynomialTrajectoryGenerator() {
  // TODO Auto-generated destructor stub
}

/**
 * @Input: ([0, 10, 0], [10, 10, 0], 1)
 * @output: [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
 *
 */
VectorXd PolynomialTrajectoryGenerator::generate_jerk_minimized_trajectory(VectorXd start, VectorXd end, double T) {

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

  VectorXd answer(6);
  answer << a0, a1, a2, a[0], a[1], a[2];
  return answer;
}

