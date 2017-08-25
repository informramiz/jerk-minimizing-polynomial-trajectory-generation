/*
 * Constants.h
 *
 *  Created on: Aug 25, 2017
 *      Author: ramiz
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <vector>
using namespace std;

class Constants {
public:
  static const int N_SAMPLES = 10;
  static const vector<double> SIGMA_S;
  static const vector<double> SIGMA_D;
  constexpr static const double SIGMA_T = 2.0;

  constexpr static const double MAX_JERK = 10; // m/s^3
  constexpr static const double MAX_ACCELERATION = 10; // m/s^2
  constexpr static const double EXPECTED_JERK_IN_ONE_SEC = 2; //m/s^2
  constexpr static const double EXPECTED_ACCELERATION_IN_ONE_SEC = 1; // m/s

  constexpr static const double SPEED_LIMIT = 30;
  constexpr static const double VEHICLE_RADIUS = 1.5; //model vehicle as circle to simplify collision detection
};

const vector<double> Constants::SIGMA_S = {10.0, 4.0, 2.0}; //[s, s_dot, s_dot_dot]
const vector<double> Constants::SIGMA_D = {1.0, 1.0, 1.0}; //[d, d_dot, d_dot_dot]

#endif /* CONSTANTS_H_ */
