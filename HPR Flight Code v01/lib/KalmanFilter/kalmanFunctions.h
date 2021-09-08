#ifndef KALMANFUNCTIONS_H
#define KALMANFUNCTIONS_H

#include "structsFC.h"
#include "constantsFC.h"
#include <Eigen.h>
#include <Eigen/Dense>

using namespace Eigen;

// This is to hold all the functions for the Kalman Filtering

// use this to initialize the currentState struct
void initializeStateVector(StateVector& stateVector, const Sensor& sensor);
//      stateVector should be your defined currentState struct that will be holding the current state of the vehicle
//      sensor is your Sensor struct holding the latest sensor data

// use this to create non-changing ricotti matricies
Ricotti buildRicotti(const constants::Noise& noise);
//      noise is a struct that comes from the constants header file, needs to be manually updated for current flight configuration

// creates the xBar vector and F matrix, need to be created ahead of time and passed by reference since c++ functions can't return more than one value
void createXBar_F(StateVector& xBar, Matrix<double, 16,16>& kalmanF,  const uint32_t& stepsize, const Sensor& sensor, const StateVector& currentState);
//      xBar is a StateVector object that will be holding the xBar values, create it as nonconst before calling this function
//      kalmanF is a 16x16 matrix that will hold the F matrix generated by the functions you must initialize as Matrix<double, 16,16> until I can get templates working
//      stepsize, time between current step and last step in millis
//      sensor is a Sensor struct holding the current sensor data
//      currentState is a StateVector holding the current values for the vehicle

// creates the zHat vector and H matrix, it also needs to know if the GPS is available
void createZHat_H(MeasurementVector& zHat,  Matrix<double, 9, 16>& kalmanH, const StateVector& xBar, const bool& GPS, const double& magDeclination);
//      zHat is a MeasurementVector to hold the zHat values, zHat being the preicted measurements
//      kalmanH is a 9x16 matrix to hold the generated H matrix. It needs to be defined as: Matrix<double, 9, 16>
//      xBar is the StateVector holding the values generated by createXBar_F
//      GPS is a boolean value that should be TRUE if GPS data is available this loop

// performs the Ricotti equations for the Kalman Filter
Matrix<double, 16,9> performRicotti(Ricotti& ricotti, const Matrix<double, 16,16>& kalmanF, const Matrix<double, 9, 16>& kalmanH, const uint32_t& stepsize);
//      ricotti takes the previously created ricotti matricies
//      kalmanF is the previously created kalmanF matrix
//      kalmanH is the previously created kalmanH matrix
//      stepsize is the elapsed milliseconds between states
//      currentState is the StateVector holding the current state
//      sensor is the Sensor struct holding the current sensor values
//
//      returns the gain (K) matrix

#endif