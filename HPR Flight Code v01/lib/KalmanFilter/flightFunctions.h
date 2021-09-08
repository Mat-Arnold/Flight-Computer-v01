#ifndef FLIGHTFUNCTIONS_H
#define FLIGHTFUNCTIONS_H

#include <structsFC.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// Checks flight phase and updates global phase variable
void phaseCheck(Phase& phase, const StateVector& currentState, const Sensor& sensor);
// phase is the master phase variable, to be updated
// currentState is the currentState vector
// sensor is the current sensor data. Want to be able to check accelerations


// fires drogue if rocket is high enough
void drogueFire(const Phase& phase, const StateVector& currentState);

// fires main if rocket is high enough
void mainFire(const Phase& phase, const StateVector& currentState);

// runs datalogging
void datalog(const Phase& phase, const StateVector& currentState, const Sensor& sensor, SDClass& builtInSD, SDClass& flashSD);


#endif