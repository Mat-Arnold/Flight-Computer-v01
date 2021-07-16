#include <Arduino.h>
#include <Eigen.h>
#include <Eigen/Dense>
#include <cmath>
#include <Streaming.h>
#include "quatFunctions.h"
#include "handyFunctions.h"
#include "constantsFC.h"
#include "structsFC.h"
#include "kalmanFunctions.h"
#include "kalmanFilter.h"


// This Kalman filter is mainly contained in the other .cpp and .h files, this main.cpp is mostly used for testing and should not be copied into main flight software
// Kalman filter requres adjusted sensor data, ie position needs to be in meters from launch already

using namespace Eigen;

Sensor sensor;
StateVector currentState;
constants::Noise noise;

void setup() 
{
  Serial.begin(115200);
  delay(5000);
  
  sensor.acc << 35.0694, 6.8646e-5, 2.3901e-5;
  sensor.gyro << -0.0022, -0.0022, -0.0102;
  sensor.mag << 0.1710, -0.1736, 0.9698;
  sensor.vel << 0.0998, -0.0925, -0.3726;
  sensor.pos << 0.0261, 3.3548, 4.4721;

  initializeStateVector(currentState, sensor);

  Ricotti ricotti{buildRicotti(noise)};
  bool GPS{true};

  Serial << "Tick: " << micros() << "\n";

  kalmanFilter(currentState, ricotti, sensor, GPS);
  uint32_t tock {micros()};
  Serial << "Tock: " << tock << "\n";
  
  
  printMatrix(currentState.quat);
  printMatrix(currentState.pos);
  printMatrix(currentState.vel);
  
  

}

void loop() {
  // put your main code here, to run repeatedly:
}