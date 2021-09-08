#include <sensorfunctions.h>
#include <structsFC.h>
#include <constantsFC.h>
#include <Adafruit_GPS.h>
#include <Eigen.h>
#include <Eigen/Dense>
#include <handyFunctions.h>
#include <cmath>
#include <Streaming.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>




Sensor sensorBuild(const bool& gpsAvail, const Adafruit_GPS& GPS, const double& initLatitude, const double& initLongitude, Adafruit_BMP3XX& bmp,const sensors_event_t& accel, const sensors_event_t& gyro, const Adafruit_LIS3MDL& mag)
{
    Sensor sensor;



    if (gpsAvail)
    {
        // Collect and convert GPS data, do it all on GPS
        // Get position data
        double xpos{};
        double ypos{};
        static double lastZpos{0};
        static uint32_t lastMillis{0};

        if (GPS.lat == 'N')
        {
            // find difference from initial, then convert to meters
            xpos = GPS.latitudeDegrees - initLatitude;
        }
        else
        {
            xpos = -1 * GPS.latitudeDegrees - initLatitude;
        }

        sensor.pos(0) = (111132.92 - 559.82 * cos(deg2rad(2 * GPS.latitudeDegrees)) + 1.175*cos(deg2rad(4 * GPS.latitudeDegrees)) - 0.0023*cos(deg2rad(6 * GPS.latitudeDegrees))) * xpos;  

        if (GPS.lon == 'E')
        {
            ypos = GPS.longitudeDegrees - initLongitude;
        }
        else
        {
            ypos = -1 * GPS.longitudeDegrees - initLongitude;
        }

        sensor.pos(1) = (111412.84 * cos(deg2rad(GPS.latitudeDegrees)) - 95.5 * cos(deg2rad(3 * GPS.latitudeDegrees)) + 0.118*cos(deg2rad(5 * GPS.latitudeDegrees))) * ypos;

        // get barometer altitude to average the two
        sensor.pos(2) = -1 * (GPS.altitude + bmp.readAltitude(constants::seaLevelPressure))/2;

        // velocity calcs

        double groundSpeed{0.514444 * GPS.speed};
        sensor.vel(0) = groundSpeed * cos(deg2rad(GPS.angle));
        sensor.vel(1) = groundSpeed * sin(deg2rad(GPS.angle));

        if(lastZpos != 0.0)
        {
            sensor.vel(2) = (sensor.pos(2) - lastZpos)/((millis() - lastMillis)/1000.00);
        }
        else
        {
            sensor.vel(2) = 0.0;
        }
        lastZpos = sensor.pos(2);
        lastMillis = millis();
        

    }
    else // if no GPS use barometer data
    {
        static double lastZposBMP{0.0};
        static uint32_t lastMillisBMP{0};

        sensor.pos(2) = -1 * bmp.readAltitude(constants::seaLevelPressure);
        if (lastZposBMP != 0)
        {
            sensor.vel(2) = (sensor.pos(2) - lastZposBMP)/ ((millis() - lastMillisBMP)/1000.00);
        }
        lastZposBMP = sensor.vel(2);
        lastMillisBMP = millis();
    }

    // Assign IMU data
    sensor.acc(0) = accel.acceleration.x;   // m/s^2
    sensor.acc(1) = accel.acceleration.y;
    sensor.acc(2) = accel.acceleration.z;
    sensor.gyro(0) = gyro.gyro.x;           // rad/s
    sensor.gyro(1) = gyro.gyro.y;
    sensor.gyro(2) = gyro.gyro.z;

    // Assign Magnetometer data
    sensor.mag(0) = mag.x;
    sensor.mag(1) = mag.y;
    sensor.mag(2) = mag.z;

    // normalize the mag vector
    sensor.mag = sensor.mag.array()/sensor.mag.norm();


    return sensor;
}