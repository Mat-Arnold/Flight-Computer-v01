% Project 2 run script

% WARNING, AT AROUND 38 SECONDS ODE45 STARTS TO ITERATE AT MINIMUM STEP
% SIZE SEVERELY PROLONGING RUN TIME, GENERATING 39 MILLION STEPS FOR THE
% 100 SECOND INTERVAL. THUS I'M FORCING IT TO A 0.01 STEP SIZE, WHICH IS
% STILL SLOW BUT A LOT FASTER.

%clear;
%clc;

% State Matrix is of the form:
% 1)  t         -s
% 2)  u         -ft/s
% 3)  v         -ft/s
% 4)  w         -ft/s
% 5)  p         -rad/s
% 6)  q         -rad/s
% 7)  r         -rad/s
% 8)  lam0
% 9)  lam1
% 10) lam2
% 11) lam3
% 12) x         -ft
% 13) y         -ft
% 14) z         -ft



% set symbolic variables for later functionalization
syms t

% constants

global g rhoAtSeaLevel launchAlt;
g = 32.2; %ft/s^2
rhoAtSeaLevel = 23.769e-4; % lb-s^2/ft^4

% ----Input Values----

% Time Vector
time = 0:0.01:30; % initial time : time step size : end time --s

% Vehicle Properties
weight = 3200; % lbs
Ixx = 16000;    % slugs-ft^2
Iyy = 6000;    % slugs-ft^2
Izz = 20000;   % slugs-ft^2
Ixz = -400;    % slugs-ft^2
%%
% --Initial Conditions--

% launch site altitude
launchAlt = 0; % ft

% relative velocity
u_int = 0.3 * 1116; % mach 0.3 ft/s --x
v_int = 0;          % ft/s          --y
w_int = 0;          % ft/s          --z

Vel_int = [u_int; v_int; w_int];  % collect velocities

% relative rotation
p_int = 0; % rad/s --roll
q_int = 0; % rad/s --pitch
r_int = 0; % rad/s --yaw

AngVel_int = [p_int;q_int;r_int]; % collect rotation

% euler angles
phi_int = 0;   % rad -- roll
theta_int = 0; % rad --pitch
psi_int = 0;   % rad -- yaw

EulerAng_int = [phi_int; theta_int; psi_int]; % Collected angles

% position
x_int = 0; % ft
y_int = 0; % ft
z_int = 0; % ft

Pos_int = [x_int; y_int; z_int]; % collected position
%%
% -- Applied Forces --
% gravity is implemented within the constructor
% all forces are body axis aligned

% -- Linear Forces--

Thrust = 300;   % lb in body frame
phiT = 0;       % thrust angle to body x-axis
Drag = 640;    % lb in stability frame
Lift = 6400;   % lb in stability frame


% -- Moments--

l = 300*cos(0.6*t); % rolling moment -- lb ft
m = 200*sin(t);     % pitching moment -- lb ft
n = -100;           % yawing moment -- lb ft

M = [l;m;n]; % Collected Moments
%%
% convert euler angles to quaterinon
quat_int = Euler2Quat(EulerAng_int);

% create initial value vector
initialValues = [Vel_int; AngVel_int; quat_int'; Pos_int];

% convert weight to mass
mass = weight/g; % slug

% build mass moment of inertia tensor
I = [Ixx 0 -Ixz;
     0  Iyy  0;
     -Ixz 0 Izz];
 

% build equations of motion
stateFunction2 = EOM_Arnold_Mat_M1(Thrust,phiT,Lift,Drag,M,mass,I);
%{ 

%%

% run through integrator
% this integrated will be good for generating sensor data
[stateMatrix, sensorOutputs] = RK4_integrator(stateFunction, time, initialValues);

% tested out a variable step RK for my own use later
stateMatrixTest = RKF45_integrator(stateFunction, [time(1) time(end)], initialValues);

% check against ode45 (forcing time step for comparison purposes)
% note that ode45 doesn't normalize quaternion
[time_ode45, stateMatrix_ode45] = ode45(stateFunction, time, initialValues);

% check difference percent for x variable
% errX = ((stateMatrix(12,:)-stateMatrix_ode45(:,11)')./stateMatrix_ode45(:,11)')*100;
% errX = errX(2:end);  % gets rid of NaN from initial conditions
%%
% checking 3D position
% figure(1)
% plot3(stateMatrix(12,:), stateMatrix(13,:), -stateMatrix(14,:));

%{
% plot comparison
figure(1)
plot(time(2:end), errX, 'Linewidth', 2);
title('Difference between RK4 and ode45 for X position result');
xlabel('Time (s)');
ylabel('Percent Difference (%)');
grid on
%}

% -- Results Plots --

% translational velocities
figure(2)
title('Linear Velocities')
subplot(3,1,1)
plot(time_ode45, stateMatrix_ode45(:,1),'b', 'Linewidth',3)
hold on
plot(stateMatrix(1,:), stateMatrix(2,:),'r', 'Linewidth', 2)
plot(stateMatrixTest(1,:),stateMatrixTest(2,:),'g', 'Linewidth',1)
title('X body-axis velocity vs time');
ylabel('(ft/s)')
xlabel('(s)')
grid on
hold off
subplot(3,1,2)
plot(time_ode45, stateMatrix_ode45(:,2),'b', 'Linewidth',3)
hold on
plot(stateMatrix(1,:), stateMatrix(3,:),'r', 'Linewidth', 2)
plot(stateMatrixTest(1,:),stateMatrixTest(3,:),'g', 'Linewidth',1)
title('Y body-axis velocity vs time');
ylabel('(ft/s)')
xlabel('(s)')
grid on
hold off
subplot(3,1,3)
plot(time_ode45, stateMatrix_ode45(:,3),'b', 'Linewidth',3)
hold on
plot(stateMatrix(1,:), stateMatrix(4,:),'r', 'Linewidth', 2)
plot(stateMatrixTest(1,:),stateMatrixTest(4,:),'g', 'Linewidth',1)
legend('ode45','RK4','RKF45','Location','Best')
title('Z body-axis velocity vs time');
ylabel('(ft/s)')
xlabel('(s)')
grid on
hold off

% rotational velocites
figure(3)
title("Rotational Velocities")
subplot(3,1,1)
plot(time_ode45, stateMatrix_ode45(:,4),'b', 'Linewidth',3)
hold on
plot(stateMatrix(1,:), stateMatrix(5,:),'r', 'Linewidth', 2)
plot(stateMatrixTest(1,:),stateMatrixTest(5,:),'g', 'Linewidth',1)
title('X body-axis rotational velocity vs time');
ylabel('(rad/s)')
xlabel('(s)')
grid on
hold off
subplot(3,1,2)
plot(time_ode45, stateMatrix_ode45(:,5),'b', 'Linewidth',3)
hold on
plot(stateMatrix(1,:), stateMatrix(6,:),'r', 'Linewidth', 2)
plot(stateMatrixTest(1,:),stateMatrixTest(6,:),'g', 'Linewidth',1)
title('Y body-axis rotational velocity vs time');
ylabel('(rad/s)')
xlabel('(s)')
grid on
hold off
subplot(3,1,3)
plot(time_ode45, stateMatrix_ode45(:,6),'b', 'Linewidth',3)
hold on
plot(stateMatrix(1,:), stateMatrix(7,:),'r', 'Linewidth', 2)
plot(stateMatrixTest(1,:),stateMatrixTest(7,:),'g', 'Linewidth',1)
legend('ode45','RK4','RKF45','Location','Best')
title('Z body-axis rotational velocity vs time');
ylabel('(rad/s)')
xlabel('(s)')
grid on
hold off

% convert quaternions to euler angles:
matlabQuats = quaternion(stateMatrix(8:11,:)');
eulers2 = quat2eul(matlabQuats); % note this gives eulers in reverse
eulersRK4 = Quat2Euler(stateMatrix(8:11,:)'); % my own function
eulersODE45 = Quat2Euler(stateMatrix_ode45(:,7:10));
eulersRKF45 = Quat2Euler(stateMatrixTest(8:11,:)');

% Euler Angles
figure(4)
title('Euler Angles')
subplot(3,1,1)
plot(time_ode45(15:end),eulersODE45(15:end,1),'b','Linewidth',3);
hold on
plot(time(2:end),eulersRK4(2:end,1),'r','Linewidth',2);
plot(stateMatrixTest(1,2:end),eulersRKF45(2:end,1),'g','Linewidth',1)
title('Phi')
xlabel('(s)')
ylabel('(rads)')
grid on
hold off
subplot(3,1,2)
plot(time_ode45,eulersODE45(:,2),'b','Linewidth',3);
hold on
plot(time,eulersRK4(:,2),'r','Linewidth',2);
plot(stateMatrixTest(1,:),eulersRKF45(:,2),'g','Linewidth',1)
title('Theta')
xlabel('(s)')
ylabel('(rads)')
grid on
hold off
subplot(3,1,3)
plot(time_ode45,eulersODE45(:,3),'b','Linewidth',3);
hold on
plot(time,eulersRK4(:,3),'r','Linewidth',2);
plot(stateMatrixTest(1,:),eulersRKF45(:,3),'g','Linewidth',1)
legend('ode45','RK4','RKF45','Location','Best')
title('Psi')
xlabel('(s)')
ylabel('(rads)')
grid on
hold off

% figure(5)
% plot(time,-stateMatrix(14,:))

 %}
