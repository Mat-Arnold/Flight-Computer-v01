function dotVector = stateFunction(t,stateVector,vehicle)
% Function that calculates the equations of motion, to be used in
% simulation integration
%
%   Global variables are utilized to not pass unchanging values to a
%   function each iteration

u = stateVector(1);
v = stateVector(2);
w = stateVector(3);
p = stateVector(4);
q = stateVector(5);
r = stateVector(6);
lam0 = stateVector(7);
lam1 = stateVector(8);
lam2 = stateVector(9);
lam3 = stateVector(10);
x = stateVector(11);
y = stateVector(12);
z = stateVector(13);

% normalize quaternions
lam0 = lam0/norm([lam0 lam1 lam2 lam3]);
lam1 = lam1/norm([lam0 lam1 lam2 lam3]);
lam2 = lam2/norm([lam0 lam1 lam2 lam3]);
lam3 = lam3/norm([lam0 lam1 lam2 lam3]);

% global launchAlt mass fuelMass totalImpulse com vehicleLength motorLength S c I Cl thrustCurve phiT
% global noseLength nFins finSpan finRootChord finTipChord finStart finThickness 
global g rhoAtSeaLevel debugValues SpeedOfSoundSL

launchAlt = vehicle.launchAlt;
mass = vehicle.mass;
fuelMass = vehicle.prop.fuelMass;
totalImpulse = vehicle.prop.totalImpulse;
com = vehicle.com;
vehicleLength = vehicle.length;
motorLength = vehicle.prop.motorLength;
S = vehicle.S;
c = vehicle.c;
I = vehicle.I;
thrustCurve = vehicle.prop.thrustCurve;
phiT = vehicle.prop.phiT;
noseLength = vehicle.noseLength;
nFins = vehicle.nFins;
finSpan = vehicle.finSpan;
finRootChord = vehicle.finRootChord;
finTipChord = vehicle.finTipChord;
finStart = vehicle.finStart;
finThickness = vehicle.finThickness;



% if (z > 0)
%     z = 0;
% end

% freestream angles
if (u ~= 0)
    alpha = atan(w/u);
    beta = atan(v/u);
else
    alpha = 0;
    beta = 0;
end




% create rotation quat for stability to body, fix this
quatStab2Body = [cos(0.5*-alpha), sin(0.5*-alpha)*[0 1 0]];

% create rotation matrix from said quat
T_Stab2Body = rotFromQuat(quatStab2Body);

% calculate airspeed (here's where windspeed can be incorporated)
airspeed = sqrt(u^2 + v^2 + w^2);
MachN = airspeed/(SpeedOfSoundSL * MachLookup(-z + launchAlt));

% density
rho = rhoAtSeaLevel * SigLookup(-z + launchAlt);

% viscosity
mu = ViscosityLookup(-z + launchAlt);
% disp(mu)

% thrust interpolation from thrust curve
% as long as end of curve is appened with [0 0] this should allow for 'shut
% off' and the simulation can run indefinitely.

Thrust = interp1(thrustCurve(1,:),thrustCurve(2,:),t,'linear', 'extrap');
% disp(Thrust)



% calculate mass loss from fuel burn
% generate a new matrix to numerically integrate

impTimestep = 0.001; % impulse time step
masstime = 0:impTimestep:t;
thrustForInt = interp1(thrustCurve(1,:),thrustCurve(2,:),masstime,'linear', 'extrap');

if (t == 0)
    fuelLoss = 0;
    fuelLossLastStep = 0;
    currentMass = mass;
else
    impulse = trapz(masstime,thrustForInt);
    impulseLastStep = trapz(masstime(1:end-1),thrustForInt(1:end-1));
    fuelLoss = (fuelMass * impulse)/totalImpulse;
    fuelLossLastStep = (fuelMass * impulseLastStep)/totalImpulse;
    currentMass = mass - fuelLoss;
end

% Thrust Damping Coefficient
fuelcom = (vehicleLength - motorLength/2);
massFlow = (fuelLoss - fuelLossLastStep)/(impTimestep); % 2-point backward method
lcn = vehicleLength - com;
lcc = fuelcom - com;
Cda = massFlow*(lcn^2 - lcc^2); % damping coefficient kg m^2/s
% disp(Cda);

% calculate Y and Z forces, as well as pitch and yaw per alpha/beta
% based on "Estimating the dynamic and aerodynamic parameters of passively
% controlled high power rockets for flight simulation"

 [CMalpha, CLalpha, Cp, CMq] = CMalphaCalc(com, noseLength, nFins, finStart, vehicleLength, finSpan, finRootChord, finTipChord, c, S, alpha);
 [CNbeta, CYbeta, Cp2, CNr] = CMalphaCalc(com, noseLength, nFins, finStart, vehicleLength, finSpan, finRootChord, finTipChord, c, S, beta);
 CD = CDcalc(vehicleLength,noseLength, nFins, finSpan, finThickness, finRootChord, finTipChord, c, airspeed, rho, mu, alpha, beta);
% disp(CD);

% drag compressibility corrections
if (MachN <= 0.8)
    CD = CD/sqrt(1 - MachN^2);
elseif (MachN >= 1.1)
    CD = CD/sqrt((MachN^2) - 1);
else
    CD = CD/sqrt(1 - (0.8^2));
end

% build force and moment derivatives

Cx = -CD;
Cy = -CYbeta * beta;
Cz = -CLalpha * alpha;

    if (airspeed == 0)
        Cm = 0;
        Cn = 0;
    else
        Cm = CMalpha * alpha + -CMq * (c/(2*airspeed)) * q ; 
        Cn =  -CNbeta * beta  + -CNr * (c/(2*airspeed)) * r; 
    end

Cl = 0;

% -- Aero Forces Stability Axis --
        Fx = 0.5 * rho * airspeed^2 * S * Cx;
        Fy = 0.5 * rho * airspeed^2 * S * Cy;
        Fz = 0.5 * rho * airspeed^2 * S * Cz;

        FAero = [Fx;Fy;Fz];
%         disp(FAero)
        
% -- Aero Moments --
        L = 0.5 * rho * airspeed^2 * S * c * Cl;
        M = 0.5 * rho * airspeed^2 * S * c * Cm - Cda * p;
        N = 0.5 * rho * airspeed^2 * S * c * Cn - Cda * r;
        
        MAero = [L;M;N];


% angular velocity vector
W = [p;q;r];

% create rotation matrix from quaternion
T_B2I = rotFromQuat([lam0 lam1 lam2 lam3]);

% build linear force equations
Thrust_body = [Thrust * cos(phiT); 0; -Thrust * sin(phiT)];
aeroForces_body = T_Stab2Body * FAero;
grav = T_B2I' * [0; 0; currentMass*g];
F = Thrust_body + aeroForces_body + grav;



% build linear acceleration component
dotV = (F./currentMass) - cross(W,[u;v;w]);
% disp(F)

% build angular acceleration component
dotW = inv(I) * (MAero - cross(W,(I*W)));
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

dotVector = [dotV;dotW;dotQuat;dotX];

debugValues(:,end+1) = [t;dotV;dotW;dotQuat;dotX;F;MAero;alpha;beta;rho;mu;airspeed;FAero;Thrust_body;grav;CD;Cp];
end

