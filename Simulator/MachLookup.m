function [astar] = MachLookup(h)
%
% MachLookup finds the appropriate speed of sound ratio for a given altitude
% using the lookup table from "Introduction to Aircraft Performance,
% Selection, and Design", and linear interpolation.
%
%   h - altitude (ft)
%
%   astar - speed of sound ratio
%

% Recreate table from book
table = [0 1000 2000 3000 4000 5000 6000 7000 8000 9000 10000 11000 12000 13000 14000 15000 16000 17000 18000 19000 20000; 1 0.988 0.977 0.965 0.949 0.942 0.930 0.918 0.905 0.893 0.880 0.867 0.867 0.867 0.867 0.867 0.867 0.867 0.867 0.867 0.867];


% check if given altitude is already defined in the table
[member, loc] = ismember(h, table(1,:));
if member
    astar = table(2,loc);
    return
    % otherwise run linear interpolation
else
    astar = interp1(table(1,:), table(2,:), h, 'linear', 'extrap');
    return
end




end

