function stateFunction = EOM_Constructor(Thrust,phiT,FAero,M,mass,I)
%
% EOM_Arnold_Mat takes the forces and moments and constructs the necessary
%  state function to solve non-linear dynamics with 6-DOF
%
%       F - column vector of the forces (can use symbolic notation if syms
%       are also declared in function) in body x, y, z
%
%       M - column vector of the moments (can use symbolic notation if syms
%       are also declared in function) in body l, m, n
%
%       mass - mass of the vehicle
%
%       I - mass moment of inertia tensor of vehicle
%
%       I'm realizing I didn't need to do this with symbols but it works
%       for now. Need to do it non-symbolically later, and make my
%       statefunction able to take force values as well, like thrust. I
%       recognize this is not the best way.
%
%       I will redo it, just a regular function but I still won't need to
%       do all the cross products by hand etc. And I'll be able to
%       add force/control arguments, allowing for thrust that adjusts over 
%       time, it just won't work with ode45 that way, but that's why I have
%       RK4 and RKF45. (can even pass functions for forces, or arrays of
%       forces over time, etc)
%

syms t rho u v w p q r lam0 lam1 lam2 lam3 x y z;
global g;

% wind force values
alpha = atan(w/u);
beta = atan(v/u);                                % pulled these from paper on 6-Dof sims, can provide


% create rotation quat for stability to body
quatStab2Body = [cos(0.5*-alpha), sin(0.5*-alpha)*[0 1 0]];

% create rotation matrix from said quat
T_Stab2Body = rotFromQuat(quatStab2Body);

% angular velocity vector
W = [p;q;r];

% create rotation matrix from quaternion
T_B2I = rotFromQuat([lam0 lam1 lam2 lam3]);

% build linear force equations
Thrust = [Thrust * cos(phiT); 0; -Thrust * sin(phiT)];
aeroForces_body = T_Stab2Body * FAero;
grav = T_B2I' * [0; 0; mass*g];
F = Thrust + aeroForces_body + grav;

% build linear acceleration component
dotV = (F./mass) - cross(W,[u;v;w]);
%disp(dotV)

% build angular acceleration component
dotW = I\( M - cross(W,(I*W)));
% disp(dotW)

% build quaternion velocity

dotQuat = 0.5 .* [(-p*lam1-q*lam2-r*lam3);
                  (p*lam0-q*lam3+r*lam2);
                  (p*lam3+q*lam0-r*lam1);
                  (-p*lam2+q*lam1+r*lam0)];
              
% disp(dotQuat)

% build position velocity
dotX = T_B2I*[u;v;w];

% disp([dotV;dotW;dotQuat;dotX])

% build state equation
stateFunction = matlabFunction([dotV;dotW;dotQuat;dotX],'Vars',{t,rho,[u;v;w;p;q;r;lam0;lam1;lam2;lam3;x;y;z]});




end

