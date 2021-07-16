
clear;
clc;

global debugValues
debugValues = [];

% State Matrix is of the form:
% 1)  t         -s
% 2)  u         -m/s
% 3)  v         -m/s
% 4)  w         -m/s
% 5)  p         -m/s
% 6)  q         -m/s
% 7)  r         -m/s
% 8)  lam0
% 9)  lam1
% 10) lam2
% 11) lam3
% 12) x         -m
% 13) y         -m
% 14) z         -m



% set symbolic variables for later functionalization
% syms t rho u v w p q r lam0 lam1 lam2 lam3 x y z

% constants
global g rhoAtSeaLevel SpeedOfSoundSL magDeclination

g = 9.81; % m/s^2
rhoAtSeaLevel = 1.226; % kg/m^3
SpeedOfSoundSL = 340; % m/s
magDeclination = 0; % rad

% ----Input Values----

% Time Vector
stepsize = 0.01;
time = 0:stepsize:30; % initial time : time step size : end time --s

% Vehicle Properties
vehicle.mass = 2.9418;      % kg
vehicle.com = 0.995;    % center of mass, m from tip
vehicle.length = 1.77;  % axial length of vechicle, ((m) from tip
vehicle.S = 0.004964;          % cross sectional area (m)
vehicle.c = 0.0795;            % rocket diameter (m)


Ixx = 0.0029248;    % kg-m^2
Iyy = 0.74579;      % kg-m^2
Izz = 0.74579;      % kg-m^2
Ixz = 0;            % kg-m^2

vehicle.noseLength = 0.343;     % m
vehicle.nFins = 3;
vehicle.finSpan = 0.07;         % m
vehicle.finRootChord = 0.20;    % m
vehicle.finTipChord = 0.10;     % m
vehicle.finStart = 1.54;        % m
vehicle.finThickness = 0.00318; % m


% Propulsion Properties

% load in thrust curve
% Aerotech I285R
% vehicle.prop.thrustCurve = zeros(2,length(time));
vehicle.prop.thrustCurve(1,:) = [0.027 0.088 0.148 0.208 0.268 0.327 0.388 0.448 0.507 0.568 0.628 0.687 0.749 0.81 0.87 0.93 0.99 1.05 1.11 1.17 1.23 1.29 1.35 1.41 1.471 1.532 2];
vehicle.prop.thrustCurve(2,:) = [171.405 325.573 341.697 358.916 373.706 373.966 368.442 367.497 361.9 351.928 346.109 340.993 329.382 321.625 310.856 295.955 283.704 269.655 253.419 240.222 224.116 204.118 118.73 23.483 2.046 0 0];
vehicle.prop.phiT = 0;       % thrust angle to body x-axis
vehicle.prop.fuelMass = 0.25062; % kg
vehicle.prop.totalImpulse = trapz(vehicle.prop.thrustCurve(1,:),vehicle.prop.thrustCurve(2,:));
vehicle.prop.motorLength = 0.250; % not important with gas gas thrusters, fuel com wont change?

% launch site altitude
vehicle.launchAlt = 0; % m

% relative velocity
u_int = 0;          % m/s          --x
v_int = 0;          % m/s          --y
w_int = 0;          % m/s          --z

Vel_int = [u_int; v_int; w_int];  % collect velocities

% relative rotation
p_int = 0; % rad/s --roll
q_int = 0; % rad/s --pitch
r_int = 0; % rad/s --yaw

AngVel_int = [p_int;q_int;r_int]; % collect rotation

% euler angles at launch
phi_int = deg2rad(0);   % rad -- roll
theta_int = deg2rad(80); % rad --pitch
psi_int = deg2rad(10);   % rad -- yaw

EulerAng_int = [phi_int; theta_int; psi_int]; % Collected angles

% position
x_int = 0; % m
y_int = 0; % m
z_int = 0; % m

Pos_int = [x_int; y_int; z_int]; % collected position

%% sensor noise struct

noise.mag = sqrt((3.2e-6)^2 + 5 * 1^2);
noise.pos = sqrt((3)^2 + 5 *1^2);
noise.vel = sqrt((0.1^2) + 5 * 1^2);
noise.gyroBias = deg2rad(1);
noise.accBias = 9.81 * 10e-6;

noise.process.quat = 0.001;
noise.process.pos = 1;
noise.process.vel = 1;
noise.process.gyroBias = 1e-6;
noise.process.accBias = 1e-6;

noise.uncertainty.quat = 0.1;
noise.uncertainty.pos = 100;
noise.uncertainty.vel = 100;



%%
% -- Applied Forces --
% gravity is implemented within the constructor
% all forces are body axis aligned

% -- Linear Forces--



%%
% convert euler angles to quaterinon
quat_int = Euler2Quat(EulerAng_int);

% create initial value vector
initialValues = [Vel_int; AngVel_int; quat_int; Pos_int];

% build mass moment of inertia tensor
vehicle.I = [Ixx 0 -Ixz;
             0  Iyy  0;
             -Ixz 0 Izz];
 

% build equations of motion
% stateFunction = EOM_Constructor(Thrust,phiT,FAero,MAero,mass,I);




%%
% run through integrator
% this integrated will be good for generating sensor data
[stateMatrix, sensorOutputs] = RK4_integrator(@stateFunction, time, initialValues, vehicle);

% tested out a variable step RK for my own use later NEEDS UPDATING TO
% SUPPORT NEW STATE FUNCTION
% stateMatrixTest = RKF45_integrator(stateFunction, [time(1) time(end)], initialValues);

% check against ode45 (forcing time step for comparison purposes)
% note that ode45 doesn't normalize quaternion
%[time_ode45, stateMatrix_ode45] = ode45(stateFunction, time, initialValues);

%%
% Noise and bias sensor data

sensorOutputs = NoiseAndBias(sensorOutputs);

%%
% Kalman Filtering

FilteredOutputs = KalmanFilter(sensorOutputs, stepsize, noise);


%%


% checking 3D position
figure(1)
plot3(stateMatrix(12,:), stateMatrix(13,:), -stateMatrix(14,:));
% alt plot
figure(2)
plot(time, -stateMatrix(14,:),'b');
hold on;
% filtered plot and noisy plot
plot(FilteredOutputs.time, -FilteredOutputs.pos(3,:),'r');
plot(time(1:3000), -sensorOutputs(15,:), 'g');
hold off;
figure(10)
plot(time, stateMatrix(12,:), 'b');
hold on;
plot(FilteredOutputs.time, FilteredOutputs.pos(1,:), 'r');
plot(time(1:3000), sensorOutputs(13,:), 'g');
hold off;
figure(11)
plot(time, stateMatrix(13,:), 'b');
hold on;
plot(FilteredOutputs.time, FilteredOutputs.pos(2,:), 'r');
plot(time(1:3000), sensorOutputs(14,:), 'g');
hold off;



% -- Results Plots --

% translational velocities
figure(3)
title('Linear Velocities')
subplot(3,1,1)
plot(stateMatrix(1,:), stateMatrix(2,:),'r', 'Linewidth', 2)
title('X body-axis velocity vs time');
ylabel('(ft/s)')
xlabel('(s)')
grid on
hold off
subplot(3,1,2)
plot(stateMatrix(1,:), stateMatrix(3,:),'r', 'Linewidth', 2)
title('Y body-axis velocity vs time');
ylabel('(ft/s)')
xlabel('(s)')
grid on
hold off
subplot(3,1,3)
plot(stateMatrix(1,:), stateMatrix(4,:),'r', 'Linewidth', 2)
title('Z body-axis velocity vs time');
ylabel('(ft/s)')
xlabel('(s)')
grid on
hold off

% rotational velocites
figure(4)
title("Rotational Velocities")
subplot(3,1,1)
plot(stateMatrix(1,:), stateMatrix(5,:),'r', 'Linewidth', 2)
title('X body-axis rotational velocity vs time');
ylabel('(rad/s)')
xlabel('(s)')
grid on
hold off
subplot(3,1,2)
plot(stateMatrix(1,:), stateMatrix(6,:),'r', 'Linewidth', 2)
title('Y body-axis rotational velocity vs time');
ylabel('(rad/s)')
xlabel('(s)')
grid on
hold off
subplot(3,1,3)
plot(stateMatrix(1,:), stateMatrix(7,:),'r', 'Linewidth', 2)
title('Z body-axis rotational velocity vs time');
ylabel('(rad/s)')
xlabel('(s)')
grid on
hold off

% convert quaternions to euler angles:
matlabQuats = quaternion(stateMatrix(8:11,:)');
eulers2 = quat2eul(matlabQuats); % note this gives eulers in reverse
eulersRK4 = Quat2Euler(stateMatrix(8:11,:)'); % my own function


% Euler Angles
figure(5)
title('Euler Angles')
subplot(3,1,1)
plot(time(2:end),eulersRK4(2:end,1),'r','Linewidth',2);
title('Phi')
xlabel('(s)')
ylabel('(rads)')
grid on
hold off
subplot(3,1,2)
plot(time,eulersRK4(:,2),'r','Linewidth',2);
title('Theta')
xlabel('(s)')
ylabel('(rads)')
grid on
hold off
subplot(3,1,3)
plot(time,eulersRK4(:,3),'r','Linewidth',2);
title('Psi')
xlabel('(s)')
ylabel('(rads)')
grid on

%{
figure(6)
plot(debugValues(1,:), debugValues(21,:))
title('Alpha');

figure(7)
plot(debugValues(1,:), debugValues(35,:))
title('CD');

figure(8)
plot(debugValues(1,:), debugValues(35,:))
title('Cp');
%}

% figure(5)
% plot(time,-stateMatrix(14,:))n bbvnc     

%{
figure(9)
subplot(5,1,1)
plot(time,sensorOutputs(1,:));
hold on
plot(time,sensorOutputs(2,:));
plot(time,sensorOutputs(3,:));
title('Linear Acceleration');
hold off
subplot(5,1,2)
plot(time,sensorOutputs(4,:));
hold on
plot(time,sensorOutputs(5,:));
plot(time,sensorOutputs(6,:));
title('Rotational Velocity');
hold off
subplot(5,1,3)
plot(time,sensorOutputs(7,:));
hold on
plot(time,sensorOutputs(8,:));
plot(time,sensorOutputs(9,:));
title('Inertial Velocity');
subplot(5,1,4)
hold off
plot(time,sensorOutputs(10,:));
hold on
plot(time,sensorOutputs(11,:));
plot(time,sensorOutputs(12,:));
title('Magnetometer');
hold off
subplot(5,1,5)
plot(time,sensorOutputs(13,:));
hold on
plot(time,sensorOutputs(14,:));
plot(time,sensorOutputs(15,:));
title('Inertial Position');
hold off
%}
