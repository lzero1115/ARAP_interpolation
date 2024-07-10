//
// Created by lex on 7/10/24.
//

#ifndef ARAP_INTERPOLATION_H
#define ARAP_INTERPOLATION_H
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include <iostream>
using namespace std;

// precomputing
void computeRotationsAndTransforms(
    const Eigen::MatrixXd& V_start,
    const Eigen::MatrixXd& V_end,
    const Eigen::MatrixXi& faces, // same connectivity
    Eigen::MatrixXd& A,
    vector<Eigen::Matrix2d>& A_transforms,
    vector<Eigen::Matrix2d>& R,
    vector<Eigen::Matrix2d>& S,
    Eigen::VectorXd& angles);

// interpolation
void computeInterpolatedVertices(
    const Eigen::MatrixXd& A,
    const vector<Eigen::Matrix2d>& A_transforms,
    const vector<Eigen::Matrix2d>& R,
    const vector<Eigen::Matrix2d>& S,
    const Eigen::VectorXd& angles,
    double t,
    const Eigen::MatrixXd& V_start,
    const Eigen::MatrixXd& V_end,
    const Eigen::MatrixXi& faces,
    Eigen::MatrixXd& V_t);

#endif //ARAP_INTERPOLATION_H
