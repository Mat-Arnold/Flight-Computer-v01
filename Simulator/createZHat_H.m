function [zHat, H] = createZHat_H(xBar,magDeclination, GPS)
% ZHat builds the predicted measurements vector from the predicted states
% (xBar). It also builds the discrete measurement matrix H, for use in the
% Ricotti equations
%
%   xBar = predicted state vector (struct)
%   magDeclination = magnetic declination angle (rad)
%
%   zHat = predicted measurement vector (struct)
%   H = discrete measurment matrix
%
%   

T_B2I = rotFromQuat(xBar.quat);

T_mag2I = [ cos(-magDeclination), sin(-magDeclination), 0;
           -sin(-magDeclination), cos(-magDeclination), 0;
           0,                     0,                    0];

% gives what the magnetometer should read based on current state

magBodyFrame = T_B2I' * T_mag2I * [1;0;0]; 

% do GPS availability checking

if (GPS)
    zHat.mag = magBodyFrame;
    zHat.pos = xBar.pos;
    zHat.vel = xBar.vel;
else
    zHat.mag = magBodyFrame;
    zHat.pos = [0;0;xBar.pos(3)];
    zHat.vel = [0;0;xBar.vel(3)];
end


%%
% create H 

lam0 = xBar.quat(1);
lam1 = xBar.quat(2);
lam2 = xBar.quat(3);
lam3 = xBar.quat(4);

if (GPS)
    H = [...
        [4*conj(lam0)*cos(magDeclination) + 2*conj(lam3)*sin(magDeclination), 4*conj(lam1)*cos(magDeclination) + 2*conj(lam2)*sin(magDeclination),                                    2*conj(lam1)*sin(magDeclination),                                    2*conj(lam0)*sin(magDeclination), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        [4*conj(lam0)*sin(magDeclination) - 2*conj(lam3)*cos(magDeclination),                                    2*conj(lam2)*cos(magDeclination), 2*conj(lam1)*cos(magDeclination) + 4*conj(lam2)*sin(magDeclination),                                   -2*conj(lam0)*cos(magDeclination), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        [2*conj(lam2)*cos(magDeclination) - 2*conj(lam1)*sin(magDeclination), 2*conj(lam3)*cos(magDeclination) - 2*conj(lam0)*sin(magDeclination), 2*conj(lam0)*cos(magDeclination) + 2*conj(lam3)*sin(magDeclination), 2*conj(lam1)*cos(magDeclination) + 2*conj(lam2)*sin(magDeclination), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        [                                                                  0,                                                                   0,                                                                   0,                                                                   0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        [                                                                  0,                                                                   0,                                                                   0,                                                                   0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        [                                                                  0,                                                                   0,                                                                   0,                                                                   0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        [                                                                  0,                                                                   0,                                                                   0,                                                                   0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]
        [                                                                  0,                                                                   0,                                                                   0,                                                                   0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
        [                                                                  0,                                                                   0,                                                                   0,                                                                   0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
        ];
else
    H = [...
        [4*conj(lam0)*cos(magDeclination) + 2*conj(lam3)*sin(magDeclination), 4*conj(lam1)*cos(magDeclination) + 2*conj(lam2)*sin(magDeclination),                                    2*conj(lam1)*sin(magDeclination),                                    2*conj(lam0)*sin(magDeclination), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        [4*conj(lam0)*sin(magDeclination) - 2*conj(lam3)*cos(magDeclination),                                    2*conj(lam2)*cos(magDeclination), 2*conj(lam1)*cos(magDeclination) + 4*conj(lam2)*sin(magDeclination),                                   -2*conj(lam0)*cos(magDeclination), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        [2*conj(lam2)*cos(magDeclination) - 2*conj(lam1)*sin(magDeclination), 2*conj(lam3)*cos(magDeclination) - 2*conj(lam0)*sin(magDeclination), 2*conj(lam0)*cos(magDeclination) + 2*conj(lam3)*sin(magDeclination), 2*conj(lam1)*cos(magDeclination) + 2*conj(lam2)*sin(magDeclination), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        [                                                                  0,                                                                   0,                                                                   0,                                                                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        [                                                                  0,                                                                   0,                                                                   0,                                                                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        [                                                                  0,                                                                   0,                                                                   0,                                                                   0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        [                                                                  0,                                                                   0,                                                                   0,                                                                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        [                                                                  0,                                                                   0,                                                                   0,                                                                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        [                                                                  0,                                                                   0,                                                                   0,                                                                   0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
        ];

end

