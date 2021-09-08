#ifndef QUATFUNCTIONS_H
#define QUATFUNCTIONS_H

#include <Eigen.h>
#include <Eigen/Dense>

// This is a header file declaring functions to be used in conjuction with quaternions, they are designed to be used in conjuction with Eigen Matrix/Vector types
using namespace Eigen;

// Creates a rotation matrix from a quaternion. In general, if being created from a body position quaternion this returns a rotation from Body to Inertial
Matrix3d rotFromQuat(const Vector4d& lam);
// lam is a Eigen::Vector4d holding a quaternion that needs to be a unit vector
// it returns a 3x3 Eigen::Matrix3d to be used as a rotation matrix

// Creates a quaternion from Euler rotations
Vector4d Euler2Quat(const Vector3d& EulerVector);
// EulerVector is a vector of length 3, holding Euler angles in radians in the order: [phi, theta, psi]
// it returns a vector of length 4, holding the resulting unit quaternion

// Gives Euler angles from a quaternion
Vector3d Quat2Euler(const Vector4d& lam);
// Euler vector of the form: [phi, theta, psi]

#endif