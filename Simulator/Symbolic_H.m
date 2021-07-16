% Symbolic H creation

syms lam0 lam1 lam2 lam3 posx posy posz velx vely velz magDeclination
syms gyroBiasx gyroBiasy gyroBiasz accBiasx accBiasy accBiasz

x = [lam0; lam1; lam2; lam3; posx; posy; posz; velx; vely; velz; gyroBiasx; gyroBiasy; gyroBiasz; accBiasx; accBiasy; accBiasz];

T_B2I = rotFromQuat([lam0 lam1 lam2 lam3]);

T_mag2I = [ cos(-magDeclination) sin(-magDeclination) 0;
           -sin(-magDeclination) cos(-magDeclination) 0;
           0                     0                    0];

T_I2B = 2 .* [((lam0^2)+(lam1^2)-0.5),  (lam1*lam2+lam0*lam3),      (lam1*lam3-lam0*lam2);
              (lam1*lam2-lam0*lam3),   ((lam0^2)+(lam2^2)-0.5),    (lam2*lam3+lam0*lam1);
              (lam1*lam3+lam0*lam2),   (lam2*lam3-lam0*lam1),      ((lam0^2)+(lam3^2)-0.5)];
       
magBodyFrame = T_I2B * T_mag2I * [1;0;0];


zHat = [magBodyFrame; posx; posy; posz; velx; vely; velz];
zHat_noGPS = [magBodyFrame;0; 0; posz; 0; 0; velz];

H = jacobian(zHat, x);
H_noGPS = jacobian(zHat_noGPS,x);