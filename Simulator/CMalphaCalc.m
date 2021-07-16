function [CMalpha,CLalpha, Cp, CMq] = CMalphaCalc(com,noseL, nFins,finStart, vehicleL, finS, finRC, finTC,diameter,area, alpha)
% This will calculate the CLalpha and Cp of the rocket. Making certain assumtions,
% ideally these values will be coming from more robust sources, ie datcom
% and CFD. These simple estimations are for testing and validation.

%   com = center of mass (m) from tip
%   noseL = nose cone length (m)
%   nFins = number of fins
%   vehicleL = vehicle length (m)
%   finsS = fin span (m)
%   finRC = fin root chord (m)
%   finTC = fin tip chord (m)
%   diameter = rocket reference diameter (m)
%   area = rocket reference cross-sectional area (m^2)
%   alpha = angle of attack (rad)

%   CMalpha = coefficient of pitch derivative for angle of attack
%   Cp = center of pressure (m) from tip

if (alpha >= deg2rad(10))
    alpha = deg2rad(10);
elseif (alpha <= -deg2rad(10))
    alpha = -deg2rad(10);
end

% Nose Cone (assuming conical, ogive, or parabolic)
noseCLa = 2;
noseCp = 0.466 * noseL;
noseBodyLift = (((2/3)*(diameter * noseL))/area) * alpha;
noseBodyCp = (5/8)*noseL;

% fins
finMidChord = (finRC + finTC)/2;
Kfb = 1 + (diameter/2)/(finS + diameter/2);
finCLa = Kfb * (((4 * nFins) * (finS/diameter)^2)/(1 + sqrt(1 +((2*finMidChord)/(finRC + finTC))^2)));
finCp = finStart + (finMidChord*(finRC + 2 * finTC))/(3*(finRC + finTC)) + (1/6)*( finRC + finTC - (finRC*finTC)/(finRC + finTC) );
finArea = (finS * ((finTC + finRC)/2)) * (3/2);

% body lift
planformArea = diameter * (vehicleL - noseL);
bodyLift = (planformArea/area) * alpha;
bodyCp = (1/2)*(vehicleL);

CLalpha = noseCLa + finCLa + noseBodyLift + bodyLift;
Cp = (noseCLa * noseCp + noseBodyLift * noseBodyCp + finCLa * finCp + bodyLift * bodyCp)/CLalpha;

% change to fin area
% noseCMa = noseCLa * ((com - noseCp)/diameter) * (((2/3)*(diameter * noseL))/finArea);
% noseBodyCMa = noseBodyLift * ((com - noseBodyCp)/diameter) * (((2/3)*(diameter * noseL))/finArea);
% finCMa = finCLa * ((com - finCp)/diameter) * ((finS * ((finTC + finRC)/2))/finArea);
% bodyCMa = bodyLift * ((com - bodyCp)/diameter) * (planformArea/finArea);



CMalpha = CLalpha * ((com - Cp)/diameter);

Vh = (finArea/area)*((Cp - com)/diameter);
CMq = 2 * CLalpha * Vh *((Cp - com)/diameter);


end

