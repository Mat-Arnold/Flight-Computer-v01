#ifndef KALMANFILTER_H
#define KALMANFILTER_H

// This is the main header for the Kalman Filter, includes all required headers

#include <Arduino.h>
#include <Eigen.h>
#include <Eigen/Dense>
#include <cmath>
#include "kalmanFunctions.h"
#include "quatFunctions.h"
#include "handyFunctions.h"
#include "constantsFC.h"
#include "structsFC.h"


// Main Kalman Filter to be called each cycle. Give it the StateVector currentState to be updated
void kalmanFilter(StateVector& currentState, Ricotti& ricotti, const Sensor& sensor, const bool& GPS);
// currentState is the StateVector holding the current state information. This will be modified by the function, plan data acquisition accordingly
// ricotti is the struct with the PREVIOUSLY CREATED Riccoti matricies using buildRicottis. can't be const because P does change
// sensor is of the Sensor type, it should contain the most up to date info from the sensor, already conditioned to the proper format
// GPS is a bool for when the GPS is available





#endif