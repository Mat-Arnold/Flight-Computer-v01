function [x, sensorOutputs] = RK4_integrator(f, t, x0, vehicle)
%
% RK4_integrator uses RK4 method to solve a system of first-order initial-value
% problems in the form x' = f(t,x), x(t0) = x0.
%
%   x = RK4System(f,t, x0), where
%
%       f is an anyonymous m-dim. vector function representing f(x,u),
%       t is an (n+1)-dim. vector representing the time steps
%       x0 is an m-dim. vector representing the initial state vector,
%
%       x is an m-by-(n+1) matrix, each column the vector of solution
%       estimates at a time step.
%
%
%
%       


x(:,1) = x0;    % set the first column to the initial values
h = t(2)-t(1); n = length(t);
sensorOutputs = zeros(15,n-1); % could be initialized with inital values

for i = 1:n-1
    
    % setup work:
    % rho = rhoAtSeaLevel * SigLookup(x(end,i) + launchAlt);
    
    % RK4 Steps
    k1 = f(t(i), x(:,i), vehicle);
    k2 = f(t(i)+ h/2, x(:,i) + h*k1/2, vehicle);
    k3 = f(t(i)+ h/2, x(:,i) + h*k2/2, vehicle);
    k4 = f(t(i)+ h, x(:,i) + h*k3, vehicle);
    x(:,i+1) = x(:,i) + h * (k1 + 2*k2 + 2*k3 + k4)/6;
    
    % need to normalize quaternions, going to start with every loop:
    % it probably doesn't need to be every loop but I don't need this super
    % fast
    % x(7:10,i+1) = x(7:10,i+1)./norm(x(7:10,i+1));
  
    % ground detection:
    
    
    if (x(13,i+1) > 0)
        x(11:12,i+1) = x(11:12,i);
        x(13, i+1) = 0;
        x(1:6, i+1) = 0;
    end
    
    
    % six dof IMU outputs for sensor simulations
    % make magnetometer reading:
    
    T_B2I = rotFromQuat(x(7:10,i+1));
    mag = T_B2I' * [1;0;0];
    
    % make inertial velocities
    vel = T_B2I * x(1:3,i+1);
    
    % sensor can't sense gravity in freefall
    antiGravVec = T_B2I' * [0;0;-9.81];
    accel = k1(1:3) + antiGravVec;
    
    % dotu dotv dotw p q r
    sensorOutputs(:,i) = [accel;x(4:6,i);vel;mag;x(11:13,i)];
    
    
    
end
% include time in function output
x = [t;x];

end