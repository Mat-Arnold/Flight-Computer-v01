
#include "kalmanFilter.h"

#include <Arduino.h>
#include <Eigen.h>
#include <Eigen/Dense>
#include <cmath>
#include "kalmanFunctions.h"
#include "quatFunctions.h"
#include "handyFunctions.h"
#include "constantsFC.h"
#include "structsFC.h"

// The main kalman filter function

void kalmanFilter(StateVector& currentState, Ricotti& ricotti, const Sensor& sensor,const bool& GPS)
{
    // detect stepsize
    
    int stepsize{static_cast<int>(millis() - currentState.time)}; // stepsize is in milliseconds
    currentState.time = millis();
    
    if (stepsize < 0)
    {
        Serial.println("STEPSIZE IS LESS THAN ZERO");
    }

    //FOR TESTING ONLY NEEDS TO BE COMMENTED OUT
    stepsize = 10;

    // zero initialize every matrix because assigning << 0 doesn't always overwrite
    StateVector xBar;
    Matrix<double, 16,16> kalmanF = Matrix<double, 16,16>::Zero();
    createXBar_F(xBar, kalmanF, stepsize, sensor, currentState);
    
   

    MeasurementVector zHat;
    Matrix<double, 9,16> kalmanH = Matrix<double, 9,16>::Zero();
    createZHat_H(zHat, kalmanH, xBar, GPS);

    // build addable measurement vectors
    VectorXd zHatAll(9);
    zHatAll << zHat.mag, zHat.pos, zHat.vel;

    VectorXd zK(9);
    if(GPS)
    {
        zK << sensor.mag, sensor.pos, sensor.vel;
    }
    else
    {
        zK << sensor.mag, 0, 0, sensor.pos(2), 0, 0, sensor.vel(2);
    }

    // find residual
    VectorXd res(9);

    res = zK - zHatAll;

    Matrix<double, 16, 9> kalmanK{performRicotti(ricotti, kalmanF, kalmanH, stepsize)};

    VectorXd tempStateVector(16);
    VectorXd xBarAll(16);
    xBarAll << xBar.quat, xBar.pos, xBar.vel, xBar.gyroBias, xBar.accBias;

    tempStateVector = xBarAll + (kalmanK * res);
    currentState.quat = tempStateVector.segment(0,4).array()/tempStateVector.segment(0,4).norm();
    currentState.pos = tempStateVector.segment(4,3);
    currentState.vel = tempStateVector.segment(7,3);
    currentState.gyroBias = tempStateVector.segment(10,3);
    currentState.accBias = tempStateVector.segment(13,3);


    
    


}