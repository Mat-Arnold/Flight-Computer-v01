function [sig] = SigLookup(h)
%
% SigLookup finds the appropriate air density ratio for a given altitude
% using the lookup table from "Introduction to Aircraft Performance,
% Selection, and Design", and linear interpolation.
%
%   h - altitude (ft)
%
%   sig - air density ratio
%


% Recreate table from book
table = [0 1000 2000 3000 4000 5000 6000 7000 8000 9000 10000 11000 12000 13000 14000 15000 16000 17000 18000 19000 20000; 1 0.907 0.822 0.742 0.668 0.601 0.538 0.481 0.428 0.380 0.337 0.297 0.254 0.217 0.185 0.158 0.135 0.115 0.098 0.084 0.072];

% check if given altitude is already defined in the table (not really
% necessary but I not so performance harming I need to get rid of it)
[member, loc] = ismember(h, table(1,:));
if member
    sig = table(2,loc);
    return
    % otherwise run linear interpolation
else
    sig = interp1(table(1,:), table(2,:), h,'linear','extrap'); 
    return
end









end

