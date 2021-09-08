#include <Arduino.h>
#include "flightFunctions.h"
#include <structsFC.h>
#include <constantsFC.h>
#include <Eigen.h>
#include <Eigen/Dense>
#include <cmath>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <cstdio>

void phaseCheck(Phase& phase, const StateVector& currentState, const Sensor& sensor)
{
    switch (phase)
    {
    case phase_standby:
        // during standby, checks to see if launch has occured
        static double total{0.0};
        static int count{0};
        static bool tracking{false};
        static double xaccel{0.0};

        // work backwards
        if (xaccel > 9.81)
        {
            phase = phase_thrust;
        }
        else if(tracking) // find average over ten samples
        {
            if(count < 10)
            {
                total += sensor.acc(0);
                ++count;
            }
            else
            {
                xaccel = total/10;
                count = 0;
                tracking = false;
            }
        }
        else if(!tracking && (sensor.acc(0) > 9.81)) // if not tracking, check if tracking is needed
        {
            tracking = true;
            total += sensor.acc(0);
            ++count;
        }
        break;


    case phase_thrust:
        // during thrust, checks for burnout
        static double oldvel{0};

        // check if velocity has decreased, this signifies burnout, recall that the z axis is towards the ground
        if(currentState.vel(2) > oldvel)
        {
            phase = phase_coast;
        }

        oldvel = currentState.vel(2);

        break;


    case phase_coast:
        // during coast check for apogee, at apogee drogue deploys
        // vertical velocity always has to cross zero, this should catch that crossing
        if((currentState.vel(2) > -0.1) && (currentState.vel(2) < 0.1))
        {
            phase = phase_apogee;
        }
        break;
    case phase_apogee:
        // during apogee rocket is descending, check for main chute deploy altitude
        if(currentState.pos(2) > (-1 * constants::chuteDeployAlt))
        {
            oldvel = 0; // reset this to be used again for next phase
            count = 0;
            phase = phase_mainChute;
        }
        break;
    case phase_mainChute:
        if ((abs(currentState.vel(2) - oldvel) < 0.2) && (currentState.vel(2) < 2.0))
        {
            ++count;
        }
        if(count >= 500)
        {
            phase = phase_landed;
        }
        oldvel = currentState.vel(2);
        // rocket should be descending under main chute, check for landing
        break;
    case phase_landed:
        // rocket has landed, transfer flight data to SD card
        // sink pyro channels
        digitalWrite(pin::drogue, LOW);
        digitalWrite(pin::main, LOW);
        break;
    
    default:
        break;
    }
}

void drogueFire(const Phase& phase, const StateVector& currentState)
{
    if((phase == phase_apogee) && (currentState.pos(2) <= -30.0))
    {
        digitalWrite(pin::drogue, HIGH);
    }
}

void mainFire(const Phase& phase, const StateVector& currentState)
{
    if((phase == phase_mainChute) && (currentState.pos(2) <= -10.0))
    {
        digitalWrite(pin::main, HIGH);
    }
}

void datalog(const Phase& phase, const StateVector& currentState, const Sensor& sensor, SDClass& builtInSD, SDClass& flashSD)
{
    if(phase != phase_landed)
    {
        char dataString[500];
        std::sprintf(dataString, "%d, %lu, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f,", 
                     phase, currentState.time, currentState.quat(0), currentState.quat(1), currentState.quat(2), currentState.quat(3), currentState.pos(0), currentState.pos(1), currentState.pos(2), currentState.vel(0), currentState.vel(1),currentState.vel(2),
                     sensor.acc(0), sensor.acc(1), sensor.acc(2), sensor.gyro(0), sensor.gyro(1), sensor.gyro(2), sensor.mag(0), sensor.mag(1), sensor.mag(2), sensor.pos(0), sensor.pos(1), sensor.pos(2), sensor.vel(0), sensor.vel(1), sensor.vel(2));
        
        File dataFile = flashSD.open("datalog.csv", FILE_WRITE);

        if(dataFile)
        {
            dataFile.println(dataString);
            dataFile.close();
        }
        else
        {
            Serial.println("Error opening datafile."); // remove for flight code
        }
    }
    else if(phase == phase_landed)
    {
        static int count{0};    // this is so it doesn't try to write to the SD a million times
        File dataFile = flashSD.open("datalog.csv");
        File dataCopy = builtInSD.open("datacopy.csv", FILE_WRITE);

        if (dataFile && (count < 1)) 
        {
            // read from the file until there's nothing else in it:
            while (dataFile.available()) 
            {
                dataCopy.write(dataFile.read());
            }
            // close the file:
            dataFile.close();
            dataCopy.close();
            ++count;
        } else 
        {
            // if the file didn't open, print an error:
            Serial.println("error opening datalog");
        }
        
    }
        
        
}