#include <Arduino.h>
#include "kalmanFunctions.h"
#include "structsFC.h"
#include "quatFunctions.h"
#include "handyFunctions.h"
#include <Eigen.h>
#include <Eigen/Dense>
#include <cmath>
#include "constantsFC.h"

void initializeStateVector(StateVector& stateVector, const Sensor& sensor)
{
    // estimate Euler rotations based on magnetometer data
    // assume no roll
    double phi{0};
    double theta{atan2(sensor.mag(2), sensor.mag(0))};

    // try to get a good pitch estimate
    double psi{};
    if (sensor.mag(0) >= 0)
    {
        psi = atan2(-1 * sensor.mag(1), sqrt(pow(sensor.mag(0),2) + pow(sensor.mag(2),2)));
    }
    else
    {
        psi = atan2(-1 * sensor.mag(1), -1 * sqrt(pow(sensor.mag(0),2) + pow(sensor.mag(2),2)));
    }

    // convert to quaternion
    Vector3d EigenVector; 
    EigenVector << phi, theta, psi;

    stateVector.quat = Euler2Quat(EigenVector);

    // position and velocity initialize to zero for origin at launch site.
    stateVector.pos = Vector3d::Zero();
    stateVector.vel = Vector3d::Zero();
    stateVector.gyroBias = Vector3d::Zero();
    stateVector.accBias = Vector3d::Zero();
    stateVector.time = millis(); // set to time at initialization in millis

}

Ricotti buildRicotti(const constants::Noise& noise)
{
    Ricotti ricotti;

    // Measurement noise matrix
    VectorXd Rvector(9);
    Rvector << noise.mag, noise.mag, noise.mag, noise.pos, noise.pos, noise.pos, noise.vel, noise.vel, noise.vel;
    Rvector = Rvector.array().pow(2);  //.array() is what makes this coefficient-wise. equavalent to .^ in MATLAB
    
    for(int i{0}; i < 9; ++i) // make sure all diagonals are nonzero
    {
        if(abs(Rvector(i)) < 1e-3)
        {
            Rvector(i) = 1e-3;
        }
    }
    ricotti.R = Rvector.asDiagonal();

    // Process Noise Matrix
    VectorXd Qvector(constants::nStates);
    Qvector << noise.processQuat, noise.processQuat, noise.processQuat, noise.processQuat, noise.processPos, noise.processPos, noise.processPos, noise.processVel, noise.processVel, noise.processVel, noise.processGyroBias, noise.processGyroBias, noise.processGyroBias, noise.processAccBias, noise.processAccBias, noise.processAccBias;
    Qvector = Qvector.array().pow(2);

    for(int i{0}; i < constants::nStates; ++i)
    {
        if(abs(Qvector(i)) < 1e-3)
        {
            Qvector(i) = 1e-3;
        }
    }
    ricotti.Q = Qvector.asDiagonal();

    // initial Covariance Matrix
    VectorXd Pvector(constants::nStates);
    Pvector << noise.uncertaintyQuat, noise.uncertaintyQuat, noise.uncertaintyQuat, noise.uncertaintyQuat, noise.uncertaintyPos, noise.uncertaintyPos, noise.uncertaintyPos, noise.uncertaintyVel, noise.uncertaintyVel, noise.uncertaintyVel, noise.gyroBias, noise.gyroBias, noise.gyroBias, noise.accBias, noise.accBias, noise.accBias;
    Pvector = Pvector.array().pow(2);

    for(int i{0}; i < constants::nStates; ++i)
    {
        if(abs(Pvector(i)) < 1e-3)
        {
            Pvector(i) = 1e-3;
        }
    }
    ricotti.P = Pvector.asDiagonal();


    return ricotti;
}

void createXBar_F(StateVector& xBar, Matrix<double, 16,16>& kalmanF,  const int& stepsize, const Sensor& sensor, const StateVector& currentState)
{
    // convert stepsize into seconds
    double stepsecs{stepsize/1000.00};
    

    // IMU data, subtract out predicted biases
    Vector3d dotBodyVel;
    dotBodyVel = sensor.acc - currentState.accBias;
    

    double p{sensor.gyro(0) - currentState.gyroBias(0)};
    double q{sensor.gyro(1) - currentState.gyroBias(1)};
    double r{sensor.gyro(2) - currentState.gyroBias(2)};



    // convert accelerations to inertial frame, and add gravity back in
    Matrix3d T_B2I;
    T_B2I = rotFromQuat(currentState.quat);

    Vector3d dotVel;
    Vector3d grav(0.0, 0.0, constants::g);
    dotVel = (T_B2I * dotBodyVel) + grav;
   

    // break out the quaternions
    double lam0{currentState.quat(0)};
    double lam1{currentState.quat(1)};
    double lam2{currentState.quat(2)};
    double lam3{currentState.quat(3)};

    // build quaternion velocity vector
    Vector4d dotQuat;
    dotQuat << (-p*lam1-q*lam2-r*lam3), (p*lam0-q*lam3+r*lam2), (p*lam3+q*lam0-r*lam1), (-p*lam2+q*lam1+r*lam0);
    dotQuat = dotQuat * 0.5;

    

    // Euler integrations to get XBar
    xBar.vel = currentState.vel + stepsecs * dotVel;
    xBar.quat = currentState.quat + stepsecs * dotQuat;
    xBar.pos = currentState.pos + stepsecs * currentState.vel;
    xBar.accBias = currentState.accBias; // biases are assumed to be constant so they have a time derivative of zero
    xBar.gyroBias = currentState.gyroBias;

    // Build the F matrix
    // This matrix was derived using the symbolic toolbox in MATLAB
    // first break out all the required variables
    double gyroBiasx{currentState.gyroBias(0)};
    double gyroBiasy{currentState.gyroBias(1)};
    double gyroBiasz{currentState.gyroBias(2)};
    double accBiasx{currentState.accBias(0)};
    double accBiasy{currentState.accBias(1)};
    double accBiasz{currentState.accBias(2)};
    double gyrox{sensor.gyro(0)};
    double gyroy{sensor.gyro(1)};
    double gyroz{sensor.gyro(2)};
    double accx{sensor.acc(0)};
    double accy{sensor.acc(1)};
    double accz{sensor.acc(2)};

    kalmanF <<                                                                       0,                                                          gyroBiasx/2 - gyrox/2,                                                          gyroBiasy/2 - gyroy/2,                                                          gyroBiasz/2 - gyroz/2, 0, 0, 0, 0, 0, 0,  lam1/2,  lam2/2,  lam3/2,                                     0,                                     0,                                     0,
                                                                 gyrox/2 - gyroBiasx/2,                                                                              0,                                                          gyroz/2 - gyroBiasz/2,                                                          gyroBiasy/2 - gyroy/2, 0, 0, 0, 0, 0, 0, -lam0/2,  lam3/2, -lam2/2,                                     0,                                     0,                                     0,
                                                                 gyroy/2 - gyroBiasy/2,                                                          gyroBiasz/2 - gyroz/2,                                                                              0,                                                          gyrox/2 - gyroBiasx/2, 0, 0, 0, 0, 0, 0, -lam3/2, -lam0/2,  lam1/2,                                     0,                                     0,                                     0,
                                                                 gyroz/2 - gyroBiasz/2,                                                          gyroy/2 - gyroBiasy/2,                                                          gyroBiasx/2 - gyrox/2,                                                                              0, 0, 0, 0, 0, 0, 0,  lam2/2, -lam1/2, -lam0/2,                                     0,                                     0,                                     0,
                                                                                     0,                                                                              0,                                                                              0,                                                                              0, 0, 0, 0, 1, 0, 0,       0,       0,       0,                                     0,                                     0,                                     0,
                                                                                     0,                                                                              0,                                                                              0,                                                                              0, 0, 0, 0, 0, 1, 0,       0,       0,       0,                                     0,                                     0,                                     0,
                                                                                     0,                                                                              0,                                                                              0,                                                                              0, 0, 0, 0, 0, 0, 1,       0,       0,       0,                                     0,                                     0,                                     0,
        4*lam0*(accx - accBiasx) - 2*lam3*(accy - accBiasy) + 2*lam2*(accz - accBiasz), 4*lam1*(accx - accBiasx) + 2*lam2*(accy - accBiasy) + 2*lam3*(accz - accBiasz),                            2*lam1*(accy - accBiasy) + 2*lam0*(accz - accBiasz),                            2*lam1*(accz - accBiasz) - 2*lam0*(accy - accBiasy), 0, 0, 0, 0, 0, 0,       0,       0,       0,    -2*pow(lam0,2) - 2*pow(lam1,2) + 1,             2*lam0*lam3 - 2*lam1*lam2,            -2*lam0*lam2 - 2*lam1*lam3,
        4*lam0*(accy - accBiasy) + 2*lam3*(accx - accBiasx) - 2*lam1*(accz - accBiasz),                            2*lam2*(accx - accBiasx) - 2*lam0*(accz - accBiasz), 2*lam1*(accx - accBiasx) + 4*lam2*(accy - accBiasy) + 2*lam3*(accz - accBiasz),                            2*lam0*(accx - accBiasx) + 2*lam2*(accz - accBiasz), 0, 0, 0, 0, 0, 0,       0,       0,       0,            -2*lam0*lam3 - 2*lam1*lam2,    -2*pow(lam0,2) - 2*pow(lam2,2) + 1,             2*lam0*lam1 - 2*lam2*lam3,
        2*lam1*(accy - accBiasy) - 2*lam2*(accx - accBiasx) + 4*lam0*(accz - accBiasz),                            2*lam0*(accy - accBiasy) + 2*lam3*(accx - accBiasx),                            2*lam3*(accy - accBiasy) - 2*lam0*(accx - accBiasx), 2*lam1*(accx - accBiasx) + 2*lam2*(accy - accBiasy) + 4*lam3*(accz - accBiasz), 0, 0, 0, 0, 0, 0,       0,       0,       0,             2*lam0*lam2 - 2*lam1*lam3,            -2*lam0*lam1 - 2*lam2*lam3,    -2*pow(lam0,2) - 2*pow(lam3,2) + 1,
                                                                                     0,                                                                              0,                                                                              0,                                                                              0, 0, 0, 0, 0, 0, 0,       0,       0,       0,                                     0,                                     0,                                     0,
                                                                                     0,                                                                              0,                                                                              0,                                                                              0, 0, 0, 0, 0, 0, 0,       0,       0,       0,                                     0,                                     0,                                     0,
                                                                                     0,                                                                              0,                                                                              0,                                                                              0, 0, 0, 0, 0, 0, 0,       0,       0,       0,                                     0,                                     0,                                     0,
                                                                                     0,                                                                              0,                                                                              0,                                                                              0, 0, 0, 0, 0, 0, 0,       0,       0,       0,                                     0,                                     0,                                     0,
                                                                                     0,                                                                              0,                                                                              0,                                                                              0, 0, 0, 0, 0, 0, 0,       0,       0,       0,                                     0,                                     0,                                     0;                                                                                                                  
}

void createZHat_H(MeasurementVector& zHat,  Matrix<double, 9, 16>& kalmanH, const StateVector& xBar, const bool& GPS)
{
    // Make a couple rotation matricies
    Matrix3d T_B2I = Matrix3d::Zero();
    Matrix3d T_mag2I = Matrix3d::Zero();

    T_B2I = rotFromQuat(xBar.quat);
    T_mag2I <<    cos(-constants::magDeclination), sin(-constants::magDeclination), 0,
               -1*sin(-constants::magDeclination), cos(-constants::magDeclination), 0,
                                                0,                               0, 0;
    
    // what the magenetometer should read based on the xBar preciction
    Vector3d magBodyFrame = Vector3d::Zero();
    Vector3d inertialMagVector(1, 0, 0);
    magBodyFrame = T_B2I.transpose() * T_mag2I * inertialMagVector;

    // check on GPS
    if(GPS)
    {
        zHat.mag = magBodyFrame;
        zHat.pos = xBar.pos;
        zHat.vel = xBar.vel;
    }
    else // we'll still have barometer data for vertical position and velocity
    {
        zHat.mag = magBodyFrame;
        zHat.pos << 0.0, 0.0, xBar.pos(2);
        zHat.vel << 0.0, 0.0, xBar.vel(2);
    }

    // now create H

    double magDeclination = constants::magDeclination;
    double lam0 = xBar.quat(0);
    double lam1 = xBar.quat(1);
    double lam2 = xBar.quat(2);
    double lam3 = xBar.quat(3);

    if (GPS)
    {
        kalmanH <<  4*lam0*cos(magDeclination) + 2*lam3*sin(magDeclination), 4*lam1*cos(magDeclination) + 2*lam2*sin(magDeclination),                              2*lam1*sin(magDeclination),                              2*lam0*sin(magDeclination), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    4*lam0*sin(magDeclination) - 2*lam3*cos(magDeclination),                              2*lam2*cos(magDeclination), 2*lam1*cos(magDeclination) + 4*lam2*sin(magDeclination),                             -2*lam0*cos(magDeclination), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    2*lam2*cos(magDeclination) - 2*lam1*sin(magDeclination), 2*lam3*cos(magDeclination) - 2*lam0*sin(magDeclination), 2*lam0*cos(magDeclination) + 2*lam3*sin(magDeclination), 2*lam1*cos(magDeclination) + 2*lam2*sin(magDeclination), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                          0,                                                       0,                                                       0,                                                       0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                          0,                                                       0,                                                       0,                                                       0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                          0,                                                       0,                                                       0,                                                       0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                          0,                                                       0,                                                       0,                                                       0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                          0,                                                       0,                                                       0,                                                       0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                                                                          0,                                                       0,                                                       0,                                                       0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
 
    }
    else
    {
        kalmanH <<  4*lam0*cos(magDeclination) + 2*lam3*sin(magDeclination), 4*lam1*cos(magDeclination) + 2*lam2*sin(magDeclination),                              2*lam1*sin(magDeclination),                              2*lam0*sin(magDeclination), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    4*lam0*sin(magDeclination) - 2*lam3*cos(magDeclination),                              2*lam2*cos(magDeclination), 2*lam1*cos(magDeclination) + 4*lam2*sin(magDeclination),                             -2*lam0*cos(magDeclination), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    2*lam2*cos(magDeclination) - 2*lam1*sin(magDeclination), 2*lam3*cos(magDeclination) - 2*lam0*sin(magDeclination), 2*lam0*cos(magDeclination) + 2*lam3*sin(magDeclination), 2*lam1*cos(magDeclination) + 2*lam2*sin(magDeclination), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                          0,                                                       0,                                                       0,                                                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                          0,                                                       0,                                                       0,                                                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                          0,                                                       0,                                                       0,                                                       0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                          0,                                                       0,                                                       0,                                                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                          0,                                                       0,                                                       0,                                                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                          0,                                                       0,                                                       0,                                                       0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
 
    }

}

Matrix<double, 16,9> performRicotti(Ricotti& ricotti, const Matrix<double, 16,16>& kalmanF, const Matrix<double, 9, 16>& kalmanH, const int& stepsize)
{
    // convert stepsize into seconds
    double stepsecs{stepsize/1000.00};
   
    // generat phiK and qK
    MatrixXd phiK(constants::nStates,constants::nStates);
    phiK = Matrix<double, constants::nStates, constants::nStates>::Identity() + kalmanF * stepsecs;

    MatrixXd qK(constants::nStates,constants::nStates);
    qK = ricotti.Q * stepsecs;

    // perform actual ricottis
    MatrixXd mK(constants::nStates,constants::nStates);
    Matrix<double, 16,9> kalmanK = Matrix<double, 16,9>::Zero();
    mK = phiK * ricotti.P * phiK.transpose() + qK;
    kalmanK = (mK * kalmanH.transpose()) * (kalmanH*mK*kalmanH.transpose() + ricotti.R).inverse();
    ricotti.P = (Matrix<double, constants::nStates, constants::nStates>::Identity() - kalmanK * kalmanH)*mK;

    return kalmanK;
  

    

}