function [noisy_sensorOutputs] = NoiseAndBias(sensorOutputs)
% NoiseAndBias adds noise and biases to the created sensor data
%   Detailed explanation goes here

udot = sensorOutputs(1,:);
vdot = sensorOutputs(2,:);
wdot = sensorOutputs(3,:);

p = sensorOutputs(4,:);
q = sensorOutputs(5,:);
r = sensorOutputs(6,:);

velx = sensorOutputs(7,:);
vely = sensorOutputs(8,:);
velz = sensorOutputs(9,:);

magx = sensorOutputs(10,:);
magy = sensorOutputs(11,:);
magz = sensorOutputs(12,:);

posx = sensorOutputs(13,:);
posy = sensorOutputs(14,:);
posz = sensorOutputs(15,:);

noisy_udot = AddNoise(udot,9.81 * 6193.6743e-9, 9.81 * 10e-6);
noisy_vdot = AddNoise(vdot,9.81 * 6193.6743e-9, 9.81 * 10e-6);
noisy_wdot = AddNoise(wdot,9.81 * 6193.6743e-9, 9.81 * 10e-6);

noisy_p = AddNoise(p,deg2rad(516.3723e-6), deg2rad(1));
noisy_q = AddNoise(q,deg2rad(516.3723e-6), deg2rad(1));
noisy_r = AddNoise(r,deg2rad(516.3723e-6), deg2rad(1));

noisy_velx = AddNoise(velx, 0.1, 0);
noisy_vely = AddNoise(vely, 0.1, 0);
noisy_velz = AddNoise(velz, 0.1, 0);

noisy_magx = AddNoise(magx, 3.2e-6, 0);
noisy_magy = AddNoise(magy, 3.2e-6, 0);
noisy_magz = AddNoise(magz, 3.2e-6, 0);

noisy_posx = AddNoise(posx, 3, 0);
noisy_posy = AddNoise(posy, 3, 0);
noisy_posz = AddNoise(posz, 3, 0);

noisy_sensorOutputs = [noisy_udot; noisy_vdot; noisy_wdot; noisy_p; noisy_q; noisy_r; noisy_velx; noisy_vely; noisy_velz; noisy_magx; noisy_magy; noisy_magz; noisy_posx; noisy_posy; noisy_posz];


end

