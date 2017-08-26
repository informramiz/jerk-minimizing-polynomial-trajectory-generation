///*
// * JMT_test.cpp
// *
// *  Created on: Aug 26, 2017
// *      Author: ramiz
// */
//
//#ifndef POLYNOMIAL_TRAJECTORY_GENERATOR_TEST_CPP_
//#define POLYNOMIAL_TRAJECTORY_GENERATOR_TEST_CPP_
//
//#include "polynomial_trajectory_generator.h"
//
//#include <iostream>
//#include <fstream>
//#include <cmath>
//#include <vector>
//#include <cassert>
//#include <iomanip>
//#include "Eigen/Dense"
//#include "vehicle.h"
//#include "utils.h"
//
//bool close_enough(vector<double> poly, vector<double> target_poly, double eps =
//                      0.01) {
//
//  if (poly.size() != target_poly.size()) {
//    cout << "your solution didn't have the correct number of terms" << endl;
//    return false;
//  }
//  for (int i = 0; i < poly.size(); i++) {
//    double diff = poly[i] - target_poly[i];
//    if (abs(diff) > eps) {
//      cout << "at least one of your terms differed from target by more than "
//          << eps << endl;
//      return false;
//    }
//
//  }
//  return true;
//}
//
//struct test_case {
//
//  vector<double> start;
//  vector<double> end;
//  double T;
//};
//
//#ifndef answers
//vector<vector<double> > answers = { { 0.0, 10.0, 0.0, 0.0, 0.0, 0.0 }, { 0.0,
//    10.0, 0.0, 0.0, -0.625, 0.3125 }, { 5.0, 10.0, 1.0, -3.0, 0.64, -0.0432 } };
//#endif
//
//void testJMT() {
//  //create test cases
//
//  vector<test_case> tc;
//
//  test_case tc1;
//  tc1.start = {0,10,0};
//  tc1.end = {10,10,0};
//  tc1.T = 1;
//  tc.push_back(tc1);
//
//  test_case tc2;
//  tc2.start = {0,10,0};
//  tc2.end = {20,15,20};
//  tc2.T = 2;
//  tc.push_back(tc2);
//
//  test_case tc3;
//  tc3.start = {5,10,2};
//  tc3.end = {-30,-20,-4};
//  tc3.T = 5;
//  tc.push_back(tc3);
//
//  bool total_correct = true;
//  for (int i = 0; i < tc.size(); i++) {
//    class PolynomialTrajectoryGenerator jmt1;
//    VectorXd generated_jmt = jmt1.generate_jerk_minimized_trajectory(Utils::vectorToVectorXd(tc[i].start), Utils::vectorToVectorXd(tc[i].end), tc[i].T);
//    bool correct = close_enough(Utils::VectorXdToVector(generated_jmt), answers[i]);
//    total_correct &= correct;
//
//  }
//  if (!total_correct) {
//    cout << "Try again!" << endl;
//  } else {
//    cout << "Nice work!" << endl;
//  }
//}
//
//#endif
