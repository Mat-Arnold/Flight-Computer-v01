#ifndef HANDYFUNCTIONS_H
#define HANDYFUNCTIONS_H
#include <Eigen.h>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace Eigen;
// for some small quality of life functions

// degrees to radians
double deg2rad(const double& x);

// radians to degrees
double rad2deg(const double& x);

// print Matrix
void printMatrix(const Ref<const MatrixXd>& mat);




#endif