// Functions to be used to manipulate or convert quaternions for Kalman filter

#include <Eigen.h>
#include <Eigen/Dense>
#include "quatFunctions.h"
#include <cmath>

Matrix3d rotFromQuat(const Vector4d& lam)
{
    //build output matrix
    
    Matrix3d output;

    output << (pow(lam(0),2) + pow(lam(1),2) - 0.5), (lam(1) * lam(2) - lam(0) * lam(3)),   (lam(1) * lam(3) + lam(0) * lam(2)),
              (lam(1) * lam(2) + lam(0) * lam(3)),   (pow(lam(0),2) + pow(lam(2),2) - 0.5), (lam(2) * lam(3) - lam(0) * lam(1)),
              (lam(1) * lam(3) - lam(0) * lam(2)),   (lam(2) * lam(3) + lam(0) * lam(1)),   (pow(lam(0),2) + pow(lam(3),2) - 0.5);

    return output * 2;
}

Vector4d Euler2Quat(const Vector3d& EulerVector)
{
    // its convenient to split out the components
    double phi{EulerVector(0)};
    double theta{EulerVector(1)};
    double psi{EulerVector(2)};

    // initialize the output quaternion
    Vector4d lam = Vector4d::Zero();

    // Build the Euler rotation matrix
    Matrix3d T;

    T << (cos(theta)*cos(psi)), (-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi)), (sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi)),
         (cos(theta)*sin(psi)), (cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi)),  (-sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi)),
         (-sin(theta)),         (sin(phi)*cos(theta)),                             (cos(phi)*cos(theta));

    
    // Get the max diagonal value and it's location
    int maxIndex{};
    double maxValue{};

    maxValue = T.diagonal().maxCoeff(&maxIndex); // a weird eigen function but it puts the index into maxIndex

    int set{};

    if (T.trace() >= maxValue)
    {
        set = 0;
    }
    else
    {
        set = maxIndex + 1;
    }
    // thus 0 for lam0 dominant, 1 for lam1 dominant etc.

    switch (set)
    {
    case 0:
        lam(0) = 0.5 * sqrt((1 + T.trace()));
        lam(1) = (1/(4*lam(0)))*(T(2,1) - T(1,2));
        lam(2) = (1/(4*lam(0)))*(T(0,2) - T(2,0));
        lam(3) = (1/(4*lam(0)))*(T(1,0) - T(0,1));
        break;
    case 1:
        lam(1) = 0.5 * sqrt((1 + 2*T(0,0) - T.trace()));
        lam(0) = (1/(4*lam(1)))*(T(2,1) - T(1,2));
        lam(2) = (1/(4*lam(1)))*(T(0,1) - T(1,0));
        lam(3) = (1/(4*lam(1)))*(T(0,2) - T(2,0));
        break;
    case 2:
        lam(2) = 0.5 * sqrt((1 + 2*T(1,1) - T.trace()));
        lam(0) = (1/(4*lam(2)))*(T(0,2) - T(2,0));
        lam(1) = (1/(4*lam(2)))*(T(0,1) - T(1,0));
        lam(3) = (1/(4*lam(2)))*(T(1,2) - T(2,1));
        break;
    case 3:
        lam(3) = 0.5 * sqrt((1 + 2*T(2,2) - T.trace()));
        lam(0) = (1/(4*lam(3)))*(T(1,0) - T(0,1));
        lam(1) = (1/(4*lam(3)))*(T(0,2) - T(2,0));
        lam(2) = (1/(4*lam(3)))*(T(1,2) - T(2,1));
        break;
    
    default:
        lam(0) = 0.5 * sqrt((1 + T.trace()));
        lam(1) = (1/(4*lam(0)))*(T(2,1) - T(1,2));
        lam(2) = (1/(4*lam(0)))*(T(0,2) - T(2,0));
        lam(3) = (1/(4*lam(0)))*(T(1,0) - T(0,1));
        break;
    }

    if (lam(0) < 0)
    {
        lam = -1 * lam;
    }

    return lam;
}

Vector3d Quat2Euler(const Vector4d& lam)
{
    Vector3d EulerVector = Vector3d::Zero();

    EulerVector(0) = atan2((2*(lam(2)*lam(3) + lam(0) * lam(1))),pow(lam(0),2) - pow(lam(1),2) - pow(lam(2),2) + pow(lam(3),2));
    EulerVector(1)= asin(2*(lam(0)*lam(2) - lam(1)*lam(3)));
    EulerVector(2) = atan2(2*(lam(1)*lam(2) + lam(0)*lam(3)), pow(lam(0),2) + pow(lam(1),2) - pow(lam(2),2) - pow(lam(3),2));
    return EulerVector;
}