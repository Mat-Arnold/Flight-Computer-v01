function mu = ViscosityLookup(h)
% Uses a table to look up air viscosity at a particular altitude
%  

alt = 1e3 .* [0 1 2 3 4 5 6 7 8 9 10 15 20 25 30 40 50 60 70 80]; % m
visc = 1e-5 .* [1.789 1.758 1.726 1.694 1.661 1.628 1.595 1.561 1.527 1.493 1.458 1.422 1.422 1.448 1.475 1.601 1.704 1.584 1.438 1.321]; % N s/m^2

mu = interp1(alt, visc, h, 'linear', 'extrap');

end

