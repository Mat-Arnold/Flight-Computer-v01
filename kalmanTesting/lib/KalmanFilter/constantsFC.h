#ifndef CONSTANTSFC_H
#define CONSTANTSFC_H

#include <cmath>
#include "handyFunctions.h"

// This is to hold the constants for the flight computer

namespace constants
{
    const double magDeclination{0}; // The magnetic declination angle in radians at launch site
    const double g{9.81};        // Acceleration due to gravity at sea level, m/s^2
    const int nStates{16};          // The number of tracked states in the state vector
    const double chuteDeployAlt{80.0};

    // this struct is to hold all the different noise values for the sensors
    struct Noise
    {
        const double mag{sqrt(pow(3.2e-6,2) + 5 * pow(1,2))};
        const double pos{sqrt(pow(3,2) + 5 * pow(1,2))};
        const double vel{sqrt(pow(0.1,2) + 5 * pow(1,2))};
        const double gyroBias{deg2rad(1)};
        const double accBias{9.81 * 10e-6};

        const double processQuat = 0.001;
        const double processPos = 0.1;
        const double processVel = 0.1;
        const double processGyroBias = 1e-6;
        const double processAccBias = 1e-6;

        const double uncertaintyQuat = 0.1;
        const double uncertaintyPos = 5;
        const double uncertaintyVel = 1;
    };
};

// use this to hold pin assignments
namespace pin
{
    const int drogue{32};
    const int main{31};
};

#endif