% call kalman filter for matlab coder

sensorOutputs = [1;2;3;4;5;6;7;8;9;10;11;12;13;14;15];
stepsize = 0.01;
noise.mag = sqrt((3.2e-6)^2 + 5 * 1^2);
noise.pos = sqrt((3)^2 + 5 *1^2);
noise.vel = sqrt((0.1^2) + 5 * 1^2);
noise.gyroBias = deg2rad(1);
noise.accBias = 9.81 * 10e-6;

noise.process.quat = 0.001;
noise.process.pos = 2;
noise.process.vel = 2;
noise.process.gyroBias = 1e-6;
noise.process.accBias = 1e-6;

noise.uncertainty.quat = 0.1;
noise.uncertainty.pos = 100;
noise.uncertainty.vel = 10;

KalmanFilter(sensorOutputs, stepsize, noise)