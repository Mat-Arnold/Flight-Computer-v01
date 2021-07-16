function [xBar, F] = createXBar_F(k, stepsize, sensor, currentState)
% advances IMU prediction for use in Kalman Filter
%
%   k = current step
%   stepsize = step size
%   sensor = struct holding the sensor data
%   currentState = struct holding the current vehicle state
%
%   xHat = predicted state


global g



% collect IMU data, subtract out predicted biases
dotBodyVel = sensor.acc(:,k) - currentState.accBias;


p = sensor.gyro(1,k) - currentState.gyroBias(1);
q = sensor.gyro(2,k) - currentState.gyroBias(2);
r = sensor.gyro(3,k) - currentState.gyroBias(3);


% get inertial accelerations, and add gravity back in
T_B2I = rotFromQuat(currentState.quat);
dotVel = (T_B2I * dotBodyVel) + [0;0;g];



% get quat velocities
lam0 = currentState.quat(1);
lam1 = currentState.quat(2);
lam2 = currentState.quat(3);
lam3 = currentState.quat(4);

dotQuat = 0.5 .* [(-p*lam1-q*lam2-r*lam3);
                  (p*lam0-q*lam3+r*lam2);
                  (p*lam3+q*lam0-r*lam1);
                  (-p*lam2+q*lam1+r*lam0)];
              
              
              
% euler integrations to get xBar

xBar.vel = currentState.vel + stepsize * dotVel;
xBar.quat = currentState.quat + stepsize * dotQuat;
xBar.pos = currentState.pos + stepsize * currentState.vel;
xBar.accBias = currentState.accBias; % assumed constant so derivative is zero
xBar.gyroBias = currentState.gyroBias;


%% build F

% posx = currentState.pos(1);
% posy = currentState.pos(2);
% posz = currentState.pos(3);
% velx = currentState.vel(1);
% vely = currentState.vel(2);
% velz = currentState.vel(3);
gyroBiasx = currentState.gyroBias(1);
gyroBiasy = currentState.gyroBias(2);
gyroBiasz = currentState.gyroBias(3);
accBiasx = currentState.accBias(1);
accBiasy = currentState.accBias(2);
accBiasz = currentState.accBias(3);
gyrox = sensor.gyro(1,k);
gyroy = sensor.gyro(2,k);
gyroz = sensor.gyro(3,k);
accx = sensor.acc(1,k);
accy = sensor.acc(2,k);
accz = sensor.acc(3,k);

F = [...
[                                                                             0,                                                          gyroBiasx/2 - gyrox/2,                                                          gyroBiasy/2 - gyroy/2,                                                          gyroBiasz/2 - gyroz/2, 0, 0, 0, 0, 0, 0,  lam1/2,  lam2/2,  lam3/2,                           0,                           0,                           0]
[                                                         gyrox/2 - gyroBiasx/2,                                                                              0,                                                          gyroz/2 - gyroBiasz/2,                                                          gyroBiasy/2 - gyroy/2, 0, 0, 0, 0, 0, 0, -lam0/2,  lam3/2, -lam2/2,                           0,                           0,                           0]
[                                                         gyroy/2 - gyroBiasy/2,                                                          gyroBiasz/2 - gyroz/2,                                                                              0,                                                          gyrox/2 - gyroBiasx/2, 0, 0, 0, 0, 0, 0, -lam3/2, -lam0/2,  lam1/2,                           0,                           0,                           0]
[                                                         gyroz/2 - gyroBiasz/2,                                                          gyroy/2 - gyroBiasy/2,                                                          gyroBiasx/2 - gyrox/2,                                                                              0, 0, 0, 0, 0, 0, 0,  lam2/2, -lam1/2, -lam0/2,                           0,                           0,                           0]
[                                                                             0,                                                                              0,                                                                              0,                                                                              0, 0, 0, 0, 1, 0, 0,       0,       0,       0,                           0,                           0,                           0]
[                                                                             0,                                                                              0,                                                                              0,                                                                              0, 0, 0, 0, 0, 1, 0,       0,       0,       0,                           0,                           0,                           0]
[                                                                             0,                                                                              0,                                                                              0,                                                                              0, 0, 0, 0, 0, 0, 1,       0,       0,       0,                           0,                           0,                           0]
[4*lam0*(accx - accBiasx) - 2*lam3*(accy - accBiasy) + 2*lam2*(accz - accBiasz), 4*lam1*(accx - accBiasx) + 2*lam2*(accy - accBiasy) + 2*lam3*(accz - accBiasz),                            2*lam1*(accy - accBiasy) + 2*lam0*(accz - accBiasz),                            2*lam1*(accz - accBiasz) - 2*lam0*(accy - accBiasy), 0, 0, 0, 0, 0, 0,       0,       0,       0,   - 2*lam0^2 - 2*lam1^2 + 1,   2*lam0*lam3 - 2*lam1*lam2, - 2*lam0*lam2 - 2*lam1*lam3]
[4*lam0*(accy - accBiasy) + 2*lam3*(accx - accBiasx) - 2*lam1*(accz - accBiasz),                            2*lam2*(accx - accBiasx) - 2*lam0*(accz - accBiasz), 2*lam1*(accx - accBiasx) + 4*lam2*(accy - accBiasy) + 2*lam3*(accz - accBiasz),                            2*lam0*(accx - accBiasx) + 2*lam2*(accz - accBiasz), 0, 0, 0, 0, 0, 0,       0,       0,       0, - 2*lam0*lam3 - 2*lam1*lam2,   - 2*lam0^2 - 2*lam2^2 + 1,   2*lam0*lam1 - 2*lam2*lam3]
[2*lam1*(accy - accBiasy) - 2*lam2*(accx - accBiasx) + 4*lam0*(accz - accBiasz),                            2*lam0*(accy - accBiasy) + 2*lam3*(accx - accBiasx),                            2*lam3*(accy - accBiasy) - 2*lam0*(accx - accBiasx), 2*lam1*(accx - accBiasx) + 2*lam2*(accy - accBiasy) + 4*lam3*(accz - accBiasz), 0, 0, 0, 0, 0, 0,       0,       0,       0,   2*lam0*lam2 - 2*lam1*lam3, - 2*lam0*lam1 - 2*lam2*lam3,   - 2*lam0^2 - 2*lam3^2 + 1]
[                                                                             0,                                                                              0,                                                                              0,                                                                              0, 0, 0, 0, 0, 0, 0,       0,       0,       0,                           0,                           0,                           0]
[                                                                             0,                                                                              0,                                                                              0,                                                                              0, 0, 0, 0, 0, 0, 0,       0,       0,       0,                           0,                           0,                           0]
[                                                                             0,                                                                              0,                                                                              0,                                                                              0, 0, 0, 0, 0, 0, 0,       0,       0,       0,                           0,                           0,                           0]
[                                                                             0,                                                                              0,                                                                              0,                                                                              0, 0, 0, 0, 0, 0, 0,       0,       0,       0,                           0,                           0,                           0]
[                                                                             0,                                                                              0,                                                                              0,                                                                              0, 0, 0, 0, 0, 0, 0,       0,       0,       0,                           0,                           0,                           0]
[                                                                             0,                                                                              0,                                                                              0,                                                                              0, 0, 0, 0, 0, 0, 0,       0,       0,       0,                           0,                           0,                           0]
];

end

