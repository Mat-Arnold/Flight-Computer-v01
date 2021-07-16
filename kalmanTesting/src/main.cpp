#include <Arduino.h>
#include "MATLABcommunicate.h"
#include "flightFunctions.h"
#include <array>
#include <Eigen.h>
#include <Eigen/Dense>
#include <KalmanFilter.h>
#include <Streaming.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

//track data cycles
std::array<double, Settings::numValues> data{};

// Kalman filter objects
using namespace Eigen;
Sensor sensor;
StateVector currentState;
constants::Noise noise;
Ricotti ricotti{buildRicotti(noise)};
Phase phase{phase_standby}; // set phase to start on standby

// SD card objects
SDClass builtInSD;
SDClass flashSD;

unsigned int loopCount{0};

void setup() 
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  // setup pyro pins
  pinMode(pin::drogue, OUTPUT);
  pinMode(pin::main, OUTPUT);
  digitalWrite(pin::drogue, LOW);
  digitalWrite(pin::drogue, LOW);

  // setup SD cards

  pinMode(BUILTIN_SDCARD,OUTPUT);
  if(!builtInSD.begin(BUILTIN_SDCARD))
  {
    Serial.println("Built in Card failed, or not present."); // will want to replace with lights/buzzer
  }

  const int flashCS{10};
  pinMode(flashCS,OUTPUT);
  digitalWrite(flashCS, HIGH);
  

  if(!flashSD.begin(flashCS))
  {
    Serial.println("FlashSD Card failed, or not present."); // will want to replace with lights/buzzer
  }

  
  // keep flash SD clean and set up column names
  flashSD.remove("datalog.csv");
  File dataFile = flashSD.open("datalog.csv", FILE_WRITE);

  if(dataFile)
  {
    dataFile.println("lam0, lam1, lam2, lam3, X, Y, Z, velX, velY, velZ,");
    dataFile.close();
  }


}

void loop() 
{
  // Wait for MATLAB handshake
  while(true)
  {
    if(handshake())
    {
      delay(100);
      data = getData();
      // Serial.println("Got Data: ");
      // for(auto number : data)
      // {
      //   Serial.println(number,6);
      // }
      break;
    }
  }
  // loop counter
  ++loopCount;
  Serial << "Loop: " << loopCount << '\n';

  // pull sensor data from array
  sensor.acc << data[0], data[1], data[2];
  sensor.gyro << data[3], data[4], data[5];
  sensor.vel << data[6], data[7], data[8];
  sensor.mag << data[9], data[10], data[11];
  sensor.pos << data[12], data[13], data[14];


  if(loopCount == 1)
  {
    initializeStateVector(currentState, sensor);
  }

  bool GPS{};
    if(loopCount % 10 == 0)
    {
      GPS = true;
    }
    else
    {
      GPS = false;
    }
  
  kalmanFilter(currentState, ricotti, sensor, GPS);

  // printMatrix(sensor.mag);
  printMatrix(currentState.quat);
  printMatrix(currentState.pos);
  printMatrix(currentState.vel);

  // Phase Testing
  phaseCheck(phase, currentState, sensor);

  Serial << phase << '\n';

  // Pyro Check
  if(phase == phase_apogee)
  {
    drogueFire(phase, currentState);
  }
  if(phase == phase_mainChute)
  {
    mainFire(phase, currentState);
  }

}
