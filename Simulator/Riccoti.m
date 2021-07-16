function [K,Pnew] = Riccoti(F,H,R,Q,P, stepsize, nStates)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

% use VanLoan method to build Phi_k and Q_k
AA = [-F Q; zeros(nStates,nStates) F']*stepsize;
BB = expm(AA);
PHIk = BB(nStates+1:2*nStates, nStates+1:2*nStates)';
Qk = PHIk * BB(1:nStates, nStates+1:2*nStates);

% do Ricotti Equations



Mk = PHIk * P * PHIk' + Qk;
K = (Mk*H')/(H*Mk*H' + R);
Pnew = (eye(nStates) - K*H)*Mk;


end

