function [FilterOutputs] = KalmanFilter(sensorOutputs, stepsize, noise)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

global magDeclination

% number of tracked states:
nStates = 16;


sensor.acc = [sensorOutputs(1,:);sensorOutputs(2,:);sensorOutputs(3,:)];

sensor.gyro = [sensorOutputs(4,:); sensorOutputs(5,:); sensorOutputs(6,:)];

sensor.vel = [sensorOutputs(7,:); sensorOutputs(8,:); sensorOutputs(9,:)];

sensor.mag = [sensorOutputs(10,:);sensorOutputs(11,:);sensorOutputs(12,:)];

sensor.pos = [sensorOutputs(13,:); sensorOutputs(14,:); sensorOutputs(15,:)];

% initilize state

% initial magnetometer reading to euler then quaternion, assuming zero roll
% on rocket

phi = 0;
theta = atan2(sensor.mag(3,1),sensor.mag(1,1));

if (sensor.mag(1,1) >= 0)
    psi = atan2(-1 * sensor.mag(2,1), sqrt(sensor.mag(1,1)^2 + sensor.mag(3,1)^2));
else
    psi = atan2(-1 * sensor.mag(2,1), -1 * sqrt(sensor.mag(1,1)^2 + sensor.mag(3,1)^2));
end


currentState.quat = Euler2Quat([phi;theta;psi]);

currentState.pos = [0;0;0]; %sensor.pos(:,1);
currentState.vel = [0;0;0];%sensor.vel(:,1);

currentState.accBias = [0;0;0];
currentState.gyroBias = [0;0;0];


t = 0;

FilterOutputs.time(1) = t;
FilterOutputs.quat(:,1) = currentState.quat;
FilterOutputs.pos(:,1) = currentState.pos;
FilterOutputs.vel(:,1) = currentState.vel;

%% build Riccati Matricies

% Measurement noise matrix
R = diag([...
         [1 1 1] * noise.mag...
         [1 1 1] * noise.pos...
         [1 1 1] * noise.vel...
         ].^2);
     
% make sure diagonals are non-zero
R = max(R,(1e-3)*eye(size(R)));

% Process noise matrix (Q)
Q = diag([...
         [1 1 1 1] * noise.process.quat...
         [1 1 1] * noise.process.pos...
         [1 1 1] * noise.process.vel...
         [1 1 1] * noise.process.gyroBias...
         [1 1 1] * noise.process.accBias...
         ].^2);
 % make sure diagonals are non-zero
 Q = max(Q,(1e-3)*eye(size(Q)));

% Make initial covariance (P)
P = diag([...
         [1 1 1 1] * noise.uncertainty.quat...
         [1 1 1] * noise.uncertainty.pos...
         [1 1 1] * noise.uncertainty.vel...
         [1 1 1] * noise.gyroBias...
         [1 1 1] * noise.accBias...
         ].^2);
     
% make sure diagonals are non-zero
P = max(P, (1e-3)*eye(size(P)));


%% perform filter loop
for k = 1:length(sensor.acc(1,:))
    % advance time step
    t = t + stepsize;
    
    
    if (mod(k,10) <= 0.001)
        GPS = 1;
    else
        GPS = 0;
    end
%     disp(GPS)
    
    % make xhat prediction using IMU data, convert to zHat as well
    [xBar, F] = createXBar_F(k, stepsize, sensor, currentState);
    [zHat, H] = createZHat_H(xBar, magDeclination, GPS);
    
   
   
    % build measurement vector, zK
    if (GPS)
        zK = [sensor.mag(:,k); sensor.pos(:,k); sensor.vel(:,k)];
    else
        zK = [sensor.mag(:,k); 0; 0;sensor.pos(3,k); 0; 0; sensor.vel(3,k)];
    end    
        
    % find residual
    res = zK - [zHat.mag; zHat.pos; zHat.vel];
    
    
    % perform Riccoti equations
    [K,P] = Riccoti(F,H,R,Q,P, stepsize, nStates);
    tempStateVector = [xBar.quat; xBar.pos; xBar.vel; xBar.gyroBias; xBar.accBias] + K * res;
    currentState.quat = tempStateVector(1:4)/norm(tempStateVector(1:4)); % normalize quaternion
    currentState.pos = tempStateVector(5:7);
    currentState.vel = tempStateVector(8:10);
    currentState.gryoBias = tempStateVector(11:13);
    currentState.accBias = tempStateVector(14:16);
    
    
    FilterOutputs.time(k) = t;
    FilterOutputs.quat(:,k) = currentState.quat;
    FilterOutputs.pos(:,k) = currentState.pos;
    FilterOutputs.vel(:,k) = currentState.vel;
end




end

