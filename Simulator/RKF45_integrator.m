function x = RKF45_integrator(f, t, x0, tol)
%
% RK45_integrator uses RKF45 variable step size method to solve a system of first-order initial-value
% problems in the form x' = f(t,x), x(t0) = x0.
%
%   x = RK4System(f,t, x0), where
%
%       f is an anyonymous m-dim. vector function representing f(x,u),
%       t is an [t-start, t-end] vector representing the time interval
%       x0 is an m-dim. vector representing the initial state vector,
%       e is tolerance
%
%       x is an m-by-(n+1) matrix, each column the vector of solution
%       estimates at a time step.
%
%
%       this still needs lots of work
%       
%       
%
if nargin < 4, tol = 1e-8; end

time = [t(1)];  % set time array
x(:,1) = x0;      % set the first column to the initial values
hmax = 0.1;
hmin = 0.0001;
h = hmin;
i = 1;
while true
    % check end condition
    if time(i) >= t(2)
        break;
    elseif (time(i) + h) >= t(2)
        h = t(2) - time(i);
    end
    
    k1 = f(time(i), x(:,i));
    k2 = f(time(i)+ h/4, x(:,i) + h*k1/4);
    k3 = f(time(i)+ 3*h/8, x(:,i) + 3*h*k1/32 + 9*h*k2/32);
    k4 = f(time(i)+ 12*h/13, x(:,i) + 1932*h*k1/2197 - 7200*h*k2/2197 + 7296*h*k3/2197);
    k5 = f(time(i) + h, x(:,i) + 439*h*k1/216 - 8*h*k2 + 3680*h*k3/513 - 845*h*k4/4104);
    k6 = f(time(i) + h/2, x(:,i) - 8*h*k1/27 + 2*h*k2 - 3544*h*k3/2565 + 1859*h*k4/4104 - 11*h*k5/40);
    x(:,i+1) = x(:,i) + h * ((25/216)*k1 + (1408/2565)*k3 + (2197/4104)*k4 - (1/5)*k4);
    
    %increment time step
    time(i+1) = time(i) + h;
    
    % need to normalize quaternions, going to start with every loop:
    % it probably doesn't need to be every loop but I don't need this super
    % fast
    x(7:10,i+1) = x(7:10,i+1)./norm(x(7:10,i+1));
    
    % non integrated values:
   
    % check adjustment
    err = (1/360)*k1 - (128/4275)*k3 - (2197/75240)*k4 + (1/50)*k5 + (2/55)*k6;
    h = 0.84 * (((tol * h)/(h*norm(err)))^(1/4)) * h;
  
    
    if h >= hmax
        h = hmax;
    elseif h <= hmin
        h = hmin;
    end
    
    i = i+1;
    
    
    
end
% include time in function output

x = [time;x];


end