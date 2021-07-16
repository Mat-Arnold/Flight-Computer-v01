% rotation matricies
syms phi theta psi;
R3 = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
R2 = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
R1 = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];