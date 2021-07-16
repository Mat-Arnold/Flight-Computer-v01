function [xnoise] = AddNoise(x,noise, bias)
% Adds noise and bias to variable (can be vector)
%   x = variable to add noise to (can be vector)
%   noise = std deviation of noise to be added
%   bias = std of bias to be added
%
%   xnoise = new noisy signal

xnoise = zeros(1,length(x));
for k = 1:length(x)
    xnoise(k) = x(k) + noise * randn;
end

totBias = bias * randn;

xnoise = xnoise + totBias;




end

