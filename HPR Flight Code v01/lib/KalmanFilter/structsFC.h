#ifndef STRUCTSFC_H
#define STRUCTSFC_H
#include <Eigen.h>
#include <Eigen/Dense>

// This is to hold the structs and objects for use in the Flight Computer

using namespace Eigen;

// Struct to collect and hold most current sensor data, all zero initialized
struct Sensor
{
    Vector3d acc = Vector3d::Zero();    // Raw acceleration from IMU, in m/s^2
    Vector3d gyro = Vector3d::Zero();   // Raw gyro data from IMU, in rad/s
    Vector3d mag = Vector3d::Zero();    // Raw magnetometer data, as unit vector
    Vector3d pos = Vector3d::Zero();    // Raw GPS position from launch, m
    Vector3d vel = Vector3d::Zero();    // GPS velocity, derivative of pos 
};

// State vector, structs of this type hold the tracked states
struct StateVector
{
    unsigned long int time{};               // Time, in milliseconds
    Vector4d quat = Vector4d::Zero();       // Attitude quaternion, unit vector
    Vector3d pos = Vector3d::Zero();        // Position, 3d coordinate in m
    Vector3d vel = Vector3d::Zero();        // Veclocity, 3d vector in m/s
    Vector3d gyroBias = Vector3d::Zero();   // gyro bias, rad/s
    Vector3d accBias = Vector3d::Zero();    // accelerometer bias, m/s^2

};

// Measurement Vector
struct MeasurementVector
{
    Vector3d mag = Vector3d::Zero();
    Vector3d pos = Vector3d::Zero();
    Vector3d vel = Vector3d::Zero();
};

struct Ricotti
{
    Matrix<double, 9, 9> R = Matrix<double, 9, 9>::Zero();
    Matrix<double, 16, 16> Q = Matrix<double, 16, 16>::Zero();
    Matrix<double, 16, 16> P = Matrix<double, 16, 16>::Zero();
};

// Phase Enumerator

enum Phase
{
    phase_standby,
    phase_thrust,
    phase_coast,
    phase_apogee,
    phase_mainChute,
    phase_landed,
};




#endif