#ifndef SENSORFUNCTIONS_H
#define SENSORFUNCTIONS_H


#include <structsFC.h>
#include <Adafruit_GPS.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>
// contains various functions for sensor reading and data aquisition


// builds sensor struct
Sensor sensorBuild(const bool& gpsAvail, const Adafruit_GPS& GPS, const double& initLatitude, const double& initLongitude, Adafruit_BMP3XX& bmp, const sensors_event_t& accel, const sensors_event_t& gyro, const Adafruit_LIS3MDL& mag);

// set up GPS
void gpsSetUp(Adafruit_GPS& GPS, double& initLatitude, double& initLongitude);


#endif