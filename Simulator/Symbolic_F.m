% symbolically create F

syms lam0 lam1 lam2 lam3 posx posy posz velx vely velz gyroBiasx gyroBiasy gyroBiasz accBiasx accBiasy accBiasz
syms gyrox gyroy gyroz accx accy accz g stepsecs processQuat processPos processVel processGyro processAcc
x = [lam0; lam1; lam2; lam3; posx; posy; posz; velx; vely; velz; gyroBiasx; gyroBiasy; gyroBiasz; accBiasx; accBiasy; accBiasz];

p = gyrox - gyroBiasx;
q = gyroy - gyroBiasy;
r = gyroz - gyroBiasz;

dotBodyVel = [accx - accBiasx; accy - accBiasy; accz - accBiasz];

T_B2I = rotFromQuat([lam0 lam1 lam2 lam3]);

dotVel = (T_B2I * dotBodyVel) + [0;0;g];

dotQuat = 0.5 .* [(-p*lam1-q*lam2-r*lam3);
                  (p*lam0-q*lam3+r*lam2);
                  (p*lam3+q*lam0-r*lam1);
                  (-p*lam2+q*lam1+r*lam0)];
              
dotPos = [velx; vely; velz];

dotBias = [0;0;0;0;0;0];

dotX = [dotQuat; dotPos; dotVel;dotBias];

F = jacobian(dotX,x);

syms a b c d e f g h i j k l m n o p

symF = [aa ab ac ad ae af ag ah ai aj ak al am an ao ap;
        ba bb bc bd be bf bg bh bi bj bk bl bm bn bo bp;
        ca cb cc bd be bf bg bh bi bj bk bl bm bn bo bp;
        da db dc bd be bf bg bh bi bj bk bl bm bn bo bp;
        ea eb ec bd be bf bg bh bi bj bk bl bm bn bo bp;
        fa fb fc bd be bf bg bh bi bj bk bl bm bn bo bp;
        ga gb gc bd be bf bg bh bi bj bk bl bm bn bo bp;
        ha hb hc bd be bf bg bh bi bj bk bl bm bn bo bp;
        ia ib ic bd be bf bg bh bi bj bk bl bm bn bo bp;
        ja jb jc bd be bf bg bh bi bj bk bl bm bn bo bp;
        ka kb kc bd be bf bg bh bi bj bk bl bm bn bo bp;
        la lb lc bd be bf bg bh bi bj bk bl bm bn bo bp;
        ma mb mc bd be bf bg bh bi bj bk bl bm bn bo bp;
        na nb nc bd be bf bg bh bi bj bk bl bm bn bo bp;
        oa ob oc bd be bf bg bh bi bj bk bl bm bn bo bp;
        pa pb pc bd be bf bg bh bi bj bk bl bm bn bo bp]

phiK = eye(16) + F * stepsecs;

Q = diag([...
         [1 1 1 1] * processQuat...
         [1 1 1] * processPos...
         [1 1 1] * processVel...
         [1 1 1] * processGyro...
         [1 1 1] * processAcc...
         ].^2);
     
temp = phiK * Q * phiK';

Qk = int(temp,stepsecs);

     