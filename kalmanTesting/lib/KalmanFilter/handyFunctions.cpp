#include <cmath>
#include <Eigen.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "handyFunctions.h"
#include <Arduino.h>
#include <Streaming.h>

using namespace Eigen;

double deg2rad(const double& x)
{
    return x * (M_PI/180);
}

double rad2deg(const double& x)
{
    return x * (180/M_PI);
}


void printMatrix(const Ref<const MatrixXd>& mat)
{
    for(int row{0}; row < mat.rows(); ++row)
    {
        for(int col{0}; col < mat.cols(); ++col)
        {
            Serial << _FLOAT(mat(row,col),6) << ' ';
        }
        Serial << '\n';
    }
}

